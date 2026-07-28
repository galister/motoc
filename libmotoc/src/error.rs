use std::{ffi::NulError, fmt, io};

use openxr as xr;

use libmonado as mnd;

#[derive(Debug)]
pub enum Error {
    Xr(xr::sys::Result),
    Monado(mnd::MndResult),
    DeviceNotFound { device: usize },
    TrackingOriginNotFound { tracking_origin: u32 },
    NoHomeDir,
    Io(io::Error),
    Json(serde_json::Error),
    Nul(NulError),
    MissingExtension(&'static str),
    DeviceNotTracked,
    InvalidRecenterSpace(String),
    HandJointLocation(xr::sys::Result),
    InvalidOperation,
    ParseFloat(std::num::ParseFloatError),
    Context(String, Box<Self>),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Xr(e) => write!(f, "OpenXR error: {:?}", e),
            Error::Monado(e) => write!(f, "Monado error: {}", e),
            Error::DeviceNotFound { device } => {
                write!(f, "no such device: {}", device)
            }
            Error::TrackingOriginNotFound { tracking_origin } => {
                write!(f, "no such tracking origin: {}", tracking_origin)
            }
            Error::NoHomeDir => write!(f, "no home dir"),
            Error::Io(e) => write!(f, "IO error: {}", e),
            Error::Json(e) => write!(f, "JSON error: {}", e),
            Error::Nul(e) => write!(f, "NUL error: {}", e),
            Error::MissingExtension(ext) => write!(f, "missing extension: {}", ext),
            Error::DeviceNotTracked => write!(f, "device not tracked"),
            Error::InvalidRecenterSpace(space) => {
                write!(f, "invalid recenter space: {}", space)
            }
            Error::HandJointLocation(e) => {
                write!(f, "failed to locate hand joints: {:?}", e)
            }
            Error::InvalidOperation => write!(f, "invalid operation"),
            Error::ParseFloat(e) => write!(f, "parse float error: {}", e),
            Error::Context(ctx, inner) => write!(f, "{}: {}", ctx, inner),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Monado(e) => Some(e),
            Error::Io(e) => Some(e),
            Error::Json(e) => Some(e),
            Error::Nul(e) => Some(e),
            Error::ParseFloat(e) => Some(e),
            Error::Context(_, inner) => Some(inner.as_ref()),
            _ => None,
        }
    }
}

impl From<xr::sys::Result> for Error {
    fn from(e: xr::sys::Result) -> Self {
        Error::Xr(e)
    }
}

impl From<mnd::MndResult> for Error {
    fn from(e: mnd::MndResult) -> Self {
        Error::Monado(e)
    }
}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Error::Json(e)
    }
}

impl From<NulError> for Error {
    fn from(e: NulError) -> Self {
        Error::Nul(e)
    }
}

impl From<std::num::ParseFloatError> for Error {
    fn from(e: std::num::ParseFloatError) -> Self {
        Error::ParseFloat(e)
    }
}

pub trait ResultExt<T> {
    fn context(self, ctx: impl Into<String>) -> Result<T, Error>;
}

impl<T> ResultExt<T> for Result<T, Error> {
    fn context(self, ctx: impl Into<String>) -> Result<T, Error> {
        self.map_err(|e| Error::Context(ctx.into(), Box::new(e)))
    }
}

impl<T> ResultExt<T> for std::result::Result<T, xr::sys::Result> {
    fn context(self, ctx: impl Into<String>) -> Result<T, Error> {
        self.map_err(|e| Error::Context(ctx.into(), Box::new(e.into())))
    }
}

impl<T> ResultExt<T> for std::result::Result<T, mnd::MndResult> {
    fn context(self, ctx: impl Into<String>) -> Result<T, Error> {
        self.map_err(|e| Error::Context(ctx.into(), Box::new(e.into())))
    }
}

impl<T> ResultExt<T> for std::result::Result<T, io::Error> {
    fn context(self, ctx: impl Into<String>) -> Result<T, Error> {
        self.map_err(|e| Error::Context(ctx.into(), Box::new(e.into())))
    }
}
