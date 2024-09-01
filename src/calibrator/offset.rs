use std::{
    f64::consts::PI,
    time::{Duration, Instant},
};

use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use nalgebra::{Rotation3, Vector3};

use crate::{
    common::UNIT,
    helpers_xr::{EffectiveSpaceVelocity, SpaceLocationConvert},
    transformd::TransformD,
};

use super::{Calibrator, StepResult};

// maintains a constant, but smoothed offset between two selected devices
pub struct OffsetMethod {
    device_a: usize,
    device_b: usize,
    target_offset: TransformD,
    lerp_factor: f64,
    spinner: Option<ProgressBar>,
    anomaly_start: Option<Instant>,
}

impl OffsetMethod {
    pub fn new_internal(a: usize, b: usize, offset: TransformD, lerp_factor: f64) -> Self {
        Self {
            device_a: a,
            device_b: b,
            target_offset: offset,
            lerp_factor,
            spinner: None,
            anomaly_start: None,
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
            anomaly_start: None,
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

        // 0.25 m/s or 35 deg/s
        if a_vel.effective_linear().norm_squared() > 0.5
            || b_vel.effective_linear().norm_squared() > 0.5
            || a_vel.effective_angular().norm_squared() > 0.6
            || b_vel.effective_angular().norm_squared() > 0.6
        {
            if let Some(spinner) = self.spinner.as_mut() {
                spinner.set_message("Device(s) moving too fast.");
                spinner.tick();
            }
            return Ok(StepResult::Continue);
        }

        let target_a = pose_b * self.target_offset;

        // TODO: implement rotation in a smarter way?
        //
        // let deviation_local = target_a.inverse() * pose_a;
        // let deviation_global = target_a.direction() * deviation_local;

        let to_b = data.get_device_origin(self.device_b)?;
        let root_b = TransformD::from(to_b.get_offset()?);

        // If user is looking far up/down, use the right unit vector as reference
        let ref_dir = if UNIT.YU.dot(&(pose_a.basis * UNIT.ZU)).abs() < 0.9 {
            UNIT.ZU
        } else {
            UNIT.XU
        };

        let v1 = target_a.basis * ref_dir;
        let v2 = pose_a.basis * ref_dir;

        let angle_y = match v2.x.atan2(v2.z) - v1.x.atan2(v1.z) {
            a if a > PI => a - 2. * PI,
            a if a < -PI => a + 2. * PI,
            a => a,
        };

        let pos_offset = root_b.origin - (target_a.origin - pose_a.origin);

        // devices are more than 100m apart → anomaly
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
            spinner.set_message("Offset mode active.");
            spinner.tick();
        }

        let offset = TransformD {
            origin: root_b.origin - (target_a.origin - pose_a.origin).scale(self.lerp_factor),
            basis: Rotation3::from_axis_angle(&UNIT.YU, angle_y * self.lerp_factor) * root_b.basis,
        };

        to_b.set_offset(offset.into())?;

        Ok(StepResult::Continue)
    }
}
