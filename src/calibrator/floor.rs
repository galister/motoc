use std::{mem::MaybeUninit, ptr};

use indicatif::{MultiProgress, ProgressBar, ProgressStyle};

use libmonado as mnd;
use openxr as xr;

use super::{Calibrator, StepResult};

// sets the floor height using palms from hand tracking
pub struct FloorMethod {
    spinner: Option<ProgressBar>,
    hands: Vec<xr::HandTracker>,
    ext_hand_tracking: xr::raw::HandTrackingEXT,
}

impl FloorMethod {
    pub fn new<G>(session: &xr::Session<G>) -> anyhow::Result<Self> {
        let mut hands = Vec::with_capacity(2);

        let Some(ext_hand_tracking) = session.instance().exts().ext_hand_tracking else {
            anyhow::bail!("EXT_hand_tracking not available!");
        };

        for h in [xr::HandEXT::LEFT, xr::HandEXT::RIGHT] {
            match session.create_hand_tracker(h) {
                Ok(hand) => hands.push(hand),
                Err(e) => log::error!("Unable to create {:?} hand tracker: {:?}", h, e),
            }
        }

        Ok(Self {
            spinner: None,
            hands,
            ext_hand_tracking,
        })
    }
}

impl Calibrator for FloorMethod {
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
        let mut lowest_y = f32::MAX;
        for hand in self.hands.iter() {
            unsafe {
                let mut locations: [xr::sys::HandJointLocationEXT; xr::HAND_JOINT_COUNT] =
                    MaybeUninit::zeroed().assume_init();

                let info = xr::sys::HandJointsLocateInfoEXT {
                    ty: xr::StructureType::HAND_JOINTS_LOCATE_INFO_EXT,
                    next: ptr::null(),
                    base_space: data.stage.as_raw(),
                    time: data.now,
                };

                let mut result = xr::sys::HandJointLocationsEXT {
                    ty: xr::StructureType::HAND_JOINT_LOCATIONS_EXT,
                    next: ptr::null_mut(),
                    is_active: xr::sys::Bool32::from_raw(0),
                    joint_count: xr::HAND_JOINT_COUNT as _,
                    joint_locations: locations.as_mut_ptr(),
                };

                let res =
                    (self.ext_hand_tracking.locate_hand_joints)(hand.as_raw(), &info, &mut result);

                if res != xr::sys::Result::SUCCESS {
                    let err: xr::Result<()> = xr::Result::Err(res);
                    anyhow::bail!("Failed to locate hand joints: {:?}", err);
                }

                let loc: &xr::HandJointLocationEXT =
                    &locations[xr::HandJointEXT::PALM.into_raw() as usize];
                if !loc.location_flags.contains(
                    xr::SpaceLocationFlags::POSITION_VALID
                        | xr::SpaceLocationFlags::POSITION_TRACKED,
                ) {
                    continue;
                }

                let low_y = loc.pose.position.y - loc.radius;
                lowest_y = lowest_y.min(low_y);
            }
        }

        if let Some(spinner) = self.spinner.as_mut() {
            if lowest_y < 100.0 {
                spinner.set_message("Running...");
            } else {
                spinner.set_message("Hands not tracking.");
            }
            spinner.tick();
        }

        if lowest_y < 0.0 {
            let mut stage = data
                .monado
                .get_reference_space_offset(mnd::ReferenceSpaceType::Stage)?;

            stage.position.y += lowest_y;
            data.monado
                .set_reference_space_offset(mnd::ReferenceSpaceType::Stage, stage)?;
        }

        Ok(StepResult::Continue)
    }
}
