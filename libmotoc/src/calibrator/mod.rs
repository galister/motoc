mod floor;
mod monitor;
mod offset;
mod recenter;
mod sampled;

pub use floor::FloorMethod;
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

#[derive(Debug, Clone)]
pub enum CalibratorStatus {
    Spinner {
        message: String,
    },
    Progress {
        current: u64,
        max: u64,
        message: String,
    },
}

pub trait Calibrator {
    fn init(&mut self, data: &mut CalibratorData) -> Result<StepResult>;
    fn step(&mut self, data: &mut CalibratorData)
        -> Result<(StepResult, Option<CalibratorStatus>)>;
    fn finish(&mut self, data: &mut CalibratorData) -> Result<()>;
}
