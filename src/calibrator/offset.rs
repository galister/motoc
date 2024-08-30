use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use nalgebra::{Rotation3, Vector3};

use crate::{helpers_xr::SpaceLocationConvert, transformd::TransformD};

use super::{Calibrator, StepResult};

// maintains a constant, but smoothed offset between two selected devices
pub struct OffsetMethod {
    device_a: usize,
    device_b: usize,
    target_offset: TransformD,
    lerp_factor: f64,
    spinner: Option<ProgressBar>,
}

impl OffsetMethod {
    pub fn new_internal(a: usize, b: usize, offset: TransformD, lerp_factor: f64) -> Self {
        Self {
            device_a: a,
            device_b: b,
            target_offset: offset,
            lerp_factor,
            spinner: None,
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
            spinner: None,
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
            data.devices[self.device_a].name
        );
        log::info!(
            "Device B: {} ({})",
            data.devices[self.device_b].serial,
            data.devices[self.device_b].name
        );

        log::info!("B-to-A offset: {}", self.target_offset);

        Ok(StepResult::Continue)
    }

    fn step(&mut self, data: &mut crate::common::CalibratorData) -> anyhow::Result<StepResult> {
        let [Ok(pose_a), Ok(pose_b)] = [
            data.devices[self.device_a]
                .space
                .locate(&data.stage, data.now)?
                .into_transformd(),
            data.devices[self.device_b]
                .space
                .locate(&data.stage, data.now)?
                .into_transformd(),
        ] else {
            if let Some(spinner) = self.spinner.as_mut() {
                spinner.set_message("Device(s) not tracking.");
                spinner.tick();
            }
            return Ok(StepResult::Continue);
        };

        if let Some(spinner) = self.spinner.as_mut() {
            spinner.set_message("Offset mode active.");
            spinner.tick();
        }

        let target_a = pose_b * self.target_offset;

        // TODO: fix rotation
        //let deviation_local = target_a.inverse() * pose_a;
        //let deviation_global = target_a.direction() * deviation_local;

        let to_b = data.get_device_origin(self.device_b)?;
        let root_b = TransformD::from(to_b.get_offset()?);

        let offset = TransformD {
            origin: root_b.origin - (target_a.origin - pose_a.origin).scale(self.lerp_factor),
            basis: root_b.basis,
        };

        to_b.set_offset(offset.into())?;

        Ok(StepResult::Continue)
    }
}
