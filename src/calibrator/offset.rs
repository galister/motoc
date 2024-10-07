use std::time::{Duration, Instant};

use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use nalgebra::{Rotation3, Vector3};

use crate::{
    helpers_xr::{EffectiveSpaceVelocity, SpaceLocationConvert},
    transformd::TransformD,
};

use libmonado_rs as mnd;

use super::{Calibrator, StepResult};

// maintains a constant, but smoothed offset between two selected devices
pub struct OffsetMethod {
    device_a: usize,
    device_b: usize,
    target_offset: TransformD,
    lerp_factor: f64,
    lerp_override_frames: u32,
    spinner: Option<ProgressBar>,
    anomaly_start: Option<Instant>,
    last_pos_a: Vector3<f64>,
}

impl OffsetMethod {
    pub fn new_internal(a: usize, b: usize, offset: TransformD, lerp_factor: f64) -> Self {
        Self {
            device_a: a,
            device_b: b,
            target_offset: offset,
            lerp_factor,
            lerp_override_frames: 0,
            spinner: None,
            anomaly_start: None,
            last_pos_a: Vector3::from_element(-1_000_000f64),
        }
    }
    pub fn new(
        a: usize,
        b: usize,
        offset_rot: Vector3<f64>,
        offset_pos: Vector3<f64>,
        lerp_factor: f64,
    ) -> Self {
        let rot = Rotation3::from_euler_angles(
            offset_rot.z.to_radians(),
            offset_rot.x.to_radians(),
            offset_rot.y.to_radians(),
        );

        Self {
            device_a: a,
            device_b: b,
            target_offset: TransformD {
                origin: offset_pos,
                basis: rot,
            },
            lerp_factor,
            lerp_override_frames: 0,
            spinner: None,
            anomaly_start: None,
            last_pos_a: Vector3::from_element(-1_000_000f64),
        }
    }
}

impl Calibrator for OffsetMethod {
    fn init(
        &mut self,
        data: &mut crate::common::CalibratorData,
        status: &mut MultiProgress,
    ) -> anyhow::Result<StepResult> {
        status.clear()?;
        let spinner = status.add(ProgressBar::new_spinner());
        spinner.set_style(ProgressStyle::default_spinner().tick_chars("⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"));

        self.spinner = Some(spinner);

        log::info!(
            "Device A: {} ({})",
            data.devices[self.device_a].serial,
            data.devices[self.device_a].inner.name
        );
        log::info!(
            "Device B: {} ({})",
            data.devices[self.device_b].serial,
            data.devices[self.device_b].inner.name
        );

        log::info!("B-to-A offset: {}", self.target_offset);

        Ok(StepResult::Continue)
    }

    fn step(&mut self, data: &mut crate::common::CalibratorData) -> anyhow::Result<StepResult> {
        let (a_loc, a_vel) = data.devices[self.device_a]
            .space
            .relate(&data.stage, data.now)?;

        let (b_loc, b_vel) = data.devices[self.device_b]
            .space
            .relate(&data.stage, data.now)?;

        let [Ok(pose_a), Ok(pose_b)] = [a_loc.into_transformd(), b_loc.into_transformd()] else {
            if let Some(spinner) = self.spinner.as_mut() {
                spinner.set_message("Device(s) not tracking.");
                spinner.tick();
            }
            return Ok(StepResult::Continue);
        };

        // 0.25 m/s or 16 deg/s
        if a_vel.effective_linear().norm_squared() > 0.5
            || b_vel.effective_linear().norm_squared() > 0.5
            || a_vel.effective_angular().norm_squared() > 0.4
            || b_vel.effective_angular().norm_squared() > 0.4
        {
            if let Some(spinner) = self.spinner.as_mut() {
                spinner.set_message("Device(s) moving too fast.");
                spinner.tick();
            }
            return Ok(StepResult::Continue);
        }

        let stage = TransformD::from(
            data.monado
                .get_reference_space_offset(mnd::ReferenceSpaceType::Stage)?,
        );

        let (pose_a, pose_b) = (stage * pose_a, stage * pose_b);

        let target_a = pose_b * self.target_offset;

        let delta_global = pose_a * target_a.inverse();

        let to_b = data.get_device_origin(self.device_b)?;
        let root_b = TransformD::from(to_b.get_offset()?);

        let pos_offset = root_b.origin + delta_global.origin;

        // devices are more than 100m from center → anomaly
        if pos_offset.norm_squared() > 10000.0 {
            if let Some(spinner) = self.spinner.as_mut() {
                // Tracker flew away
                spinner.set_message("Anomaly detected...");
                spinner.tick();
            }

            // anomaly doesn't disappear within 5s → reset offset
            match self.anomaly_start {
                Some(time) => {
                    if time.elapsed() > Duration::from_secs(5) {
                        log::info!("Tracking anomaly detected. Restarting from scratch.");
                        to_b.set_offset(TransformD::default().into())?;
                        self.anomaly_start = Some(Instant::now());
                    }
                }
                None => {
                    self.anomaly_start = Some(Instant::now());
                }
            }

            return Ok(StepResult::Continue);
        } else {
            self.anomaly_start = None;
        }

        if let Some(spinner) = self.spinner.as_mut() {
            spinner.set_message(format!(
                "Offset mode active. Deviation: {:.2}m {:.1}°",
                delta_global.origin.norm(),
                delta_global.basis.angle().to_degrees()
            ));
            spinner.tick();
        }

        let lerp_factor = if (pose_a.origin - self.last_pos_a).norm_squared() > 0.5 {
            log::info!("Tracking jump on device A, ignoring lerp factor.");
            self.lerp_override_frames = 9;
            1.0
        } else if self.lerp_override_frames > 0 {
            self.lerp_override_frames -= 1;
            1.0
        } else {
            self.lerp_factor
        };

        self.last_pos_a = pose_a.origin;

        let offset = TransformD {
            origin: root_b.origin + (delta_global.origin).scale(lerp_factor),
            basis: Rotation3::default().slerp(&delta_global.basis, self.lerp_factor) * root_b.basis,
        };

        to_b.set_offset(offset.into())?;

        Ok(StepResult::Continue)
    }
}
