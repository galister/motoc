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
use crate::error::Error;

pub type Result<T> = std::result::Result<T, Error>;

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
    ) -> Result<StepResult>;

    fn step(&mut self, data: &mut CalibratorData) -> Result<StepResult>;
    fn finish(&mut self, data: &mut CalibratorData) -> Result<()>;
}
