mod floor;
mod monitor;
mod offset;
mod recenter;
mod sampled;

pub use floor::FloorMethod;
use indicatif::MultiProgress;
pub use monitor::Monitor;
pub use offset::OffsetMethod;
pub use recenter::RecenterMethod;
pub use sampled::SampledMethod;

use crate::common::CalibratorData;

pub enum StepResult {
    // Continue running the current instance
    Continue,
    // Switch to using a different instance
    Replace(Box<dyn Calibrator>),
    // Stop calibration work
    End,
}

pub trait Calibrator {
    fn init(
        &mut self,
        data: &mut CalibratorData,
        status: &mut MultiProgress,
    ) -> anyhow::Result<StepResult>;

    fn step(&mut self, data: &mut CalibratorData) -> anyhow::Result<StepResult>;
}
