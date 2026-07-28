use libmonado as mnd;
use nalgebra::Rotation3;
use openxr as xr;

use crate::{
    common::UNIT,
    error::{Error, ResultExt},
    helpers_xr::SpaceLocationConvert,
    transformd::TransformD,
};

use super::{Calibrator, CalibratorStatus, StepResult};

pub type Result<T> = std::result::Result<T, Error>;

enum HeightMode {
    Normal,
    Keep,
    Relative(f64),
}

// sets the floor height using palms from hand tracking
pub struct RecenterMethod {
    space: xr::ReferenceSpaceType,
    height_mode: HeightMode,
}

impl RecenterMethod {
    pub fn new(space: &str, height: &Option<String>) -> Result<Self> {
        let space = match space.to_lowercase().as_str() {
            "stage" => xr::ReferenceSpaceType::STAGE,
            "local" => xr::ReferenceSpaceType::LOCAL,
            _ => return Err(Error::InvalidRecenterSpace(space.to_string())),
        };

        let height_mode = match height {
            Some(s) => {
                if s.to_lowercase() == "keep" {
                    HeightMode::Keep
                } else {
                    HeightMode::Relative(s.parse()?)
                }
            }
            None => HeightMode::Normal,
        };

        Ok(Self { space, height_mode })
    }
}

impl Calibrator for RecenterMethod {
    fn init(&mut self, _data: &mut crate::common::CalibratorData) -> Result<StepResult> {
        Ok(StepResult::Continue)
    }

    fn step(
        &mut self,
        data: &mut crate::common::CalibratorData,
    ) -> Result<(StepResult, Option<CalibratorStatus>)> {
        let (space, mnd_space) = match self.space {
            xr::ReferenceSpaceType::STAGE => (&data.stage, mnd::ReferenceSpaceType::Stage),
            xr::ReferenceSpaceType::LOCAL => (&data.local, mnd::ReferenceSpaceType::Local),
            _ => panic!("Unexpected reference space {:?}", self.space),
        };
        let loc = data
            .view
            .locate(space, data.now)
            .context("Unable to locate VIEW")?;

        let Ok(hmd) = loc.into_transformd() else {
            return Ok((
                StepResult::Continue,
                Some(CalibratorStatus::Spinner {
                    message: String::from("Device(s) not tracking."),
                }),
            ));
        };

        let current = TransformD::from(data.monado.get_reference_space_offset(mnd_space)?);
        let mut stage_offset = current;

        let horiz_hmd_pos = nalgebra::Vector3::new(hmd.origin.x, 0.0, hmd.origin.z);

        let fwd = hmd.basis * UNIT.NEG_ZU;
        let horiz_len_sq = fwd.x * fwd.x + fwd.z * fwd.z;

        let hmd_yaw = if horiz_len_sq > f64::EPSILON {
            let yaw = (-fwd.x).atan2(-fwd.z);
            Rotation3::from_axis_angle(&UNIT.YU, yaw)
        } else {
            Rotation3::identity()
        };

        let recenter_offset = TransformD {
            basis: hmd_yaw,
            origin: horiz_hmd_pos,
        };

        stage_offset = stage_offset * recenter_offset;

        let mut new_reference =
            TransformD::from(data.monado.get_reference_space_offset(mnd_space)?);
        new_reference.origin.x = stage_offset.origin.x;
        new_reference.origin.z = stage_offset.origin.z;

        if horiz_len_sq > f64::EPSILON {
            new_reference.basis = stage_offset.basis;
        }

        match self.height_mode {
            HeightMode::Keep => {
                new_reference.origin.y = current.origin.y;
            }
            HeightMode::Relative(h) => {
                new_reference.origin.y = stage_offset.origin.y - h;
            }
            _ => {}
        }

        log::info!(
            "Enjoy your new {:?} space! The values are {new_reference}",
            self.space
        );

        data.monado
            .set_reference_space_offset(mnd_space, new_reference.into())
            .context("Unable to set reference space offset")?;

        Ok((StepResult::End, None))
    }
    fn finish(&mut self, _data: &mut crate::common::CalibratorData) -> Result<()> {
        Ok(())
    }
}
