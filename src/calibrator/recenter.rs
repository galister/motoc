use indicatif::{MultiProgress, ProgressBar, ProgressStyle};

use libmonado as mnd;
use nalgebra::Rotation3;
use openxr as xr;

use crate::{common::UNIT, helpers_xr::SpaceLocationConvert, transformd::TransformD};

use super::{Calibrator, StepResult};

enum HeightMode {
    Normal,
    Keep,
    Relative(f64),
}

// sets the floor height using palms from hand tracking
pub struct RecenterMethod {
    spinner: Option<ProgressBar>,
    space: xr::ReferenceSpaceType,
    height_mode: HeightMode,
}

impl RecenterMethod {
    pub fn new(space: &str, height: &Option<String>) -> anyhow::Result<Self> {
        let space = match space.to_lowercase().as_str() {
            "stage" => xr::ReferenceSpaceType::STAGE,
            "local" => xr::ReferenceSpaceType::LOCAL,
            _ => anyhow::bail!("Can only recenter spaces LOCAL and STAGE!"),
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

        Ok(Self {
            spinner: None,
            space,
            height_mode,
        })
    }
}

impl Calibrator for RecenterMethod {
    fn init(
        &mut self,
        _data: &mut crate::common::CalibratorData,
        status: &mut MultiProgress,
    ) -> anyhow::Result<StepResult> {
        status.clear()?;
        let spinner = status.add(ProgressBar::new_spinner());
        spinner.set_style(ProgressStyle::default_spinner().tick_chars("⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"));

        Ok(StepResult::Continue)
    }

    fn step(&mut self, data: &mut crate::common::CalibratorData) -> anyhow::Result<StepResult> {
        let (space, mnd_space) = match self.space {
            xr::ReferenceSpaceType::STAGE => (&data.stage, mnd::ReferenceSpaceType::Stage),
            xr::ReferenceSpaceType::LOCAL => (&data.local, mnd::ReferenceSpaceType::Local),
            _ => panic!("Unexpected reference space {:?}", self.space),
        };
        let loc = data.view.locate(space, data.now)?;

        let Ok(hmd) = loc.into_transformd() else {
            if let Some(spinner) = self.spinner.as_mut() {
                spinner.set_message("Device(s) not tracking.");
                spinner.tick();
            }
            return Ok(StepResult::Continue);
        };

        let current = TransformD::from(data.monado.get_reference_space_offset(mnd_space)?);
        let mut hmd = current * hmd;

        let fwd = hmd.basis * UNIT.NEG_ZU;
        let yaw = fwd.x.atan2(-fwd.z);
        hmd.basis = Rotation3::from_axis_angle(&UNIT.YU, yaw);

        let mut new_reference = hmd.inverse();

        match self.height_mode {
            HeightMode::Keep => {
                new_reference.origin.y = current.origin.y;
            }
            HeightMode::Relative(h) => {
                new_reference.origin.y = hmd.origin.y - h;
            }
            _ => {}
        }

        log::info!(
            "Enjoy your new {:?} space! The values are {new_reference}",
            self.space
        );

        data.monado
            .set_reference_space_offset(mnd_space, new_reference.into())?;

        Ok(StepResult::End)
    }
}
