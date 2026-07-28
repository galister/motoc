use std::{mem::MaybeUninit, ptr};

use libmonado as mnd;
use openxr as xr;

use crate::error::{Error, ResultExt};

use super::{Calibrator, CalibratorStatus, StepResult};

pub type Result<T> = std::result::Result<T, Error>;

// sets the floor height using palms from hand tracking
pub struct FloorMethod {
    hands: Vec<xr::HandTracker>,
    ext_hand_tracking: xr::raw::HandTrackingEXT,
}

impl FloorMethod {
    pub fn new<G>(session: &xr::Session<G>) -> Result<Self> {
        let mut hands = Vec::with_capacity(2);

        let Some(ext_hand_tracking) = session.instance().exts().ext_hand_tracking else {
            return Err(Error::MissingExtension("EXT_hand_tracking"));
        };

        for h in [xr::HandEXT::LEFT, xr::HandEXT::RIGHT] {
            match session.create_hand_tracker(h) {
                Ok(hand) => hands.push(hand),
                Err(e) => log::error!("Unable to create {:?} hand tracker: {:?}", h, e),
            }
        }

        Ok(Self {
            hands,
            ext_hand_tracking,
        })
    }
}

impl Calibrator for FloorMethod {
    fn init(&mut self, _data: &mut crate::common::CalibratorData) -> Result<StepResult> {
        Ok(StepResult::Continue)
    }

    fn step(
        &mut self,
        data: &mut crate::common::CalibratorData,
    ) -> Result<(StepResult, Option<CalibratorStatus>)> {
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
                    return Err(Error::HandJointLocation(res));
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

        let status = if lowest_y < 100.0 {
            CalibratorStatus::Spinner {
                message: String::from("Running..."),
            }
        } else {
            CalibratorStatus::Spinner {
                message: String::from("Hands not tracking."),
            }
        };

        if lowest_y < 0.0 {
            let mut stage = data
                .monado
                .get_reference_space_offset(mnd::ReferenceSpaceType::Stage)
                .context("Unable to get reference offset")?;

            stage.position.y += lowest_y;
            data.monado
                .set_reference_space_offset(mnd::ReferenceSpaceType::Stage, stage)
                .context("Unable to set reference offset")?;
        }

        Ok((StepResult::Continue, Some(status)))
    }
    fn finish(&mut self, _data: &mut crate::common::CalibratorData) -> Result<()> {
        Ok(())
    }
}
