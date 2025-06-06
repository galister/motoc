use anyhow::anyhow;
use indicatif::{MultiProgress, ProgressBar};
use nalgebra::{Dyn, Matrix3, OMatrix, Rotation3, RowVector3, UnitQuaternion, Vector3, U1, U3};

use libmonado as mnd;

use crate::{
    calibrator::{OffsetMethod, StepResult},
    common::OffsetType,
    helpers_xr::SpaceLocationConvert,
    transformd::TransformD,
};

use super::Calibrator;

struct DeltaRotSample {
    a: RowVector3<f64>,
    b: RowVector3<f64>,
}

impl DeltaRotSample {
    fn new(new: &Sample, old: &Sample) -> Option<Self> {
        let delta_a = new.a.basis * old.a.basis.transpose();
        let delta_b = new.b.basis * old.b.basis.transpose();

        let angle_a = angle_from_mat3a(delta_a.matrix());
        let angle_b = angle_from_mat3a(delta_b.matrix());

        let samp_a = axis_from_mat3a(delta_a.matrix());
        let samp_b = axis_from_mat3a(delta_b.matrix());

        if angle_a < 0.4
            || angle_b < 0.4
            || samp_a.norm_squared() < 0.1
            || samp_b.norm_squared() < 0.1
        {
            None
        } else {
            Some(Self {
                a: samp_a.normalize(),
                b: samp_b.normalize(),
            })
        }
    }
}

fn axis_from_mat3a(mat: &Matrix3<f64>) -> RowVector3<f64> {
    RowVector3::new(
        mat.row(2)[1] - mat.row(1)[2],
        mat.row(0)[2] - mat.row(2)[0],
        mat.row(1)[0] - mat.row(0)[1],
    )
}

fn angle_from_mat3a(mat: &Matrix3<f64>) -> f64 {
    ((mat.row(0)[0] + mat.row(1)[1] + mat.row(2)[2] - 1.0) / 2.0).acos()
}

#[derive(Default, Clone, Copy)]
struct Sample {
    a: TransformD,
    b: TransformD,
}

/// finds the offset by sampling two devices moving together over time
///
/// implements the math from OpenVR-SpaceCalibrator by pushrax
/// https://github.com/pushrax/OpenVR-SpaceCalibrator/blob/master/math.pdf
pub struct SampledMethod {
    src_dev: usize,
    dst_dev: usize,
    samples: Vec<Sample>,
    maintain: bool,
    num_samples: usize,
    progress: Option<ProgressBar>,
    profile: String,
}

impl SampledMethod {
    pub fn new(
        src_dev: usize,
        dst_dev: usize,
        maintain: bool,
        samples: u32,
        profile: String,
    ) -> Self {
        Self {
            src_dev,
            dst_dev,
            samples: Vec::with_capacity(1000),
            maintain,
            num_samples: samples as _,
            progress: None,
            profile,
        }
    }

    fn collect_samples(&mut self, data: &mut crate::common::CalibratorData) -> anyhow::Result<()> {
        let new_a = data.devices[self.src_dev]
            .space
            .locate(&data.stage, data.now)?
            .into_transformd()?;

        let new_b = data.devices[self.dst_dev]
            .space
            .locate(&data.stage, data.now)?
            .into_transformd()?;

        let stage = TransformD::from(
            data.monado
                .get_reference_space_offset(mnd::ReferenceSpaceType::Stage)?,
        );

        let (new_a, new_b) = (stage * new_a, stage * new_b);
        self.samples.push(Sample { a: new_a, b: new_b });

        Ok(())
    }

    fn calibrate_rotation(&self) -> Rotation3<f64> {
        let mut deltas = Vec::with_capacity(self.samples.len());

        for i in 0..self.samples.len() {
            for j in 0..i {
                if let Some(delta) = DeltaRotSample::new(&self.samples[i], &self.samples[j]) {
                    deltas.push(delta);
                }
            }
        }

        log::info!(
            "Got {} samples with {} delta samples.",
            self.samples.len(),
            deltas.len()
        );

        let mut a_centroid = RowVector3::zeros();
        let mut b_centroid = RowVector3::zeros();

        for d in deltas.iter() {
            a_centroid += d.a;
            b_centroid += d.b;
        }

        let len_recip = 1.0 / deltas.len() as f64;
        a_centroid *= len_recip;
        b_centroid *= len_recip;

        let mut a_points = OMatrix::<f64, Dyn, U3>::zeros(deltas.len());
        let mut b_points = OMatrix::<f64, Dyn, U3>::zeros(deltas.len());

        for (i, d) in deltas.iter().enumerate() {
            a_points.set_row(i, &(d.a - a_centroid));
            b_points.set_row(i, &(d.b - b_centroid));
        }

        let cross_cv = a_points.transpose() * b_points;

        let svd = cross_cv.svd(true, true);

        let u = svd.u.unwrap();
        let v = svd.v_t.unwrap().transpose();

        let mut i = Matrix3::identity();

        if (u * v.transpose()).determinant() < 0.0 {
            i.row_mut(2)[2] = -1.0;
        }

        let rot = v * i * u.transpose();
        let rot = rot.transpose();

        Rotation3::from_matrix_unchecked(rot)
    }

    fn calibrate_translation(&self, rot: &Rotation3<f64>) -> anyhow::Result<Vector3<f64>> {
        let mut deltas = Vec::with_capacity(self.samples.len());

        for i in 0..self.samples.len() {
            let mut si = self.samples[i];
            si.b.basis = rot * si.b.basis;
            si.b.origin = rot * si.b.origin;

            for j in 0..i {
                let mut sj = self.samples[j];
                sj.b.basis = rot * sj.b.basis;
                sj.b.origin = rot * sj.b.origin;

                let rot_a_i = si.a.basis.transpose();
                let rot_a_j = sj.a.basis.transpose();
                let delta_rot_a = rot_a_j.matrix() - rot_a_i.matrix();

                let ca =
                    rot_a_j * (sj.a.origin - sj.b.origin) - rot_a_i * (si.a.origin - si.b.origin);
                deltas.push((ca, delta_rot_a));

                let rot_b_i = si.b.basis.transpose();
                let rot_b_j = sj.b.basis.transpose();
                let delta_rot_b = rot_b_j.matrix() - rot_b_i.matrix();

                let cb =
                    rot_b_j * (sj.a.origin - sj.b.origin) - rot_b_i * (si.a.origin - si.b.origin);
                deltas.push((cb, delta_rot_b));
            }
        }

        let mut constants = OMatrix::<f64, Dyn, U1>::zeros(deltas.len() * 3);
        let mut coeffs = OMatrix::<f64, Dyn, U3>::zeros(deltas.len() * 3);

        for i in 0..deltas.len() {
            for axis in 0..3 {
                constants[i * 3 + axis] = deltas[i].0[axis];
                coeffs.set_row(i * 3 + axis, &deltas[i].1.row(axis));
            }
        }

        coeffs
            .svd(true, true)
            .solve(&constants, f32::EPSILON as f64)
            .map_err(|e| anyhow!(e))
    }

    fn avg_b_to_a_offset(&self, offset: &TransformD) -> TransformD {
        let mut vecs = Vector3::zeros();
        let mut quat: Option<UnitQuaternion<_>> = None;

        for samp in self.samples.iter() {
            let b_to_a = (*offset * samp.b).inverse() * samp.a;

            vecs += b_to_a.origin;
            let q = UnitQuaternion::from_rotation_matrix(&b_to_a.basis);

            if let Some(quat) = quat.as_mut() {
                *quat = quat.slerp(&q, 0.1);
            } else {
                quat = Some(q);
            }
        }

        let out_pos = vecs.scale(1.0 / self.samples.len() as f64);

        TransformD {
            basis: quat.unwrap().to_rotation_matrix(),
            origin: out_pos,
        }
    }
}

impl Calibrator for SampledMethod {
    fn init(
        &mut self,
        _: &mut crate::common::CalibratorData,
        status: &mut MultiProgress,
    ) -> anyhow::Result<StepResult> {
        status.clear()?;
        self.progress = Some(status.add(ProgressBar::new(self.num_samples as _)));

        log::info!("Move the two devices together!");

        Ok(StepResult::Continue)
    }

    fn step(&mut self, data: &mut crate::common::CalibratorData) -> anyhow::Result<StepResult> {
        if self.samples.len() < self.num_samples {
            let _ = self.collect_samples(data);

            if let Some(progress) = self.progress.as_mut() {
                progress.set_message("Collecting samples...");
                progress.set_position(self.samples.len() as _);
                progress.tick();
            }

            return Ok(StepResult::Continue);
        }

        if let Some(progress) = self.progress.as_mut() {
            progress.set_message("Calculating...");
            progress.tick();
        }

        // sampling done, calculate
        let rot = self.calibrate_rotation();
        let pos = self.calibrate_translation(&rot)?;

        let dst_origin = data.get_device_origin(self.dst_dev)?;

        if pos.norm_squared() > 10000.0 {
            log::info!("Calibration failed, retrying...");
            self.samples.clear();
            dst_origin.set_offset(TransformD::default().into())?;
            return Ok(StepResult::Continue);
        }

        let offset = TransformD {
            basis: rot,
            origin: pos,
        };

        log::info!("Calibration done. Offset: {}", offset);

        let dst_root = TransformD::from(dst_origin.get_offset()?);
        let full_offset = offset * dst_root;
        dst_origin.set_offset(full_offset.into())?;

        if self.maintain {
            let offset = self.avg_b_to_a_offset(&offset);

            match data.save_calibration(
                &self.profile,
                self.src_dev,
                self.dst_dev,
                offset,
                OffsetType::Device,
            ) {
                Ok(_) => log::info!(
                    "Saved calibration. Use `motoc continue` on next startup to use this."
                ),
                Err(e) => log::warn!("Could not save calibration: {}", e),
            }

            Ok(StepResult::Replace(Box::new(OffsetMethod::new_internal(
                self.src_dev,
                self.dst_dev,
                offset,
                0.02,
            ))))
        } else {
            let src_origin = data.get_device_origin(self.src_dev)?;
            let src_root = TransformD::from(src_origin.get_offset()?);
            match data.save_calibration(
                &self.profile,
                src_origin.id as _,
                dst_origin.id as _,
                full_offset * src_root.inverse(),
                OffsetType::TrackingOrigin,
            ) {
                Ok(_) => log::info!(
                    "Saved calibration. Use `motoc continue` on next startup to use this."
                ),
                Err(e) => log::warn!("Could not save calibration: {}", e),
            }

            Ok(StepResult::End)
        }
    }
}
