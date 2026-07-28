use std::{ffi::CString, str::FromStr};

use nalgebra::Vector3;
use openxr::{self as xr};
use openxr_mndx_xdev_space::XR_MNDX_XDEV_SPACE_EXTENSION_NAME;

use crate::error::Error;
use crate::transformd::TransformD;

pub type Result<T> = std::result::Result<T, Error>;

pub fn xr_init() -> Result<(xr::Instance, xr::SystemId)> {
    let entry = xr::Entry::linked();

    let Ok(available_extensions) = entry.enumerate_extensions() else {
        return Err(Error::Context(
            "Failed to enumerate OpenXR extensions".into(),
            Box::new(Error::Xr(xr::sys::Result::ERROR_INITIALIZATION_FAILED)),
        ));
    };

    if !available_extensions.mnd_headless {
        return Err(Error::MissingExtension("MND_headless"));
    }

    let xdev_ext_name = CString::from_str(XR_MNDX_XDEV_SPACE_EXTENSION_NAME)?.into_bytes_with_nul();

    if !available_extensions.other.contains(&xdev_ext_name) {
        return Err(Error::MissingExtension("MNDX_xdev_space"));
    }

    let mut enabled_extensions = xr::ExtensionSet::default();
    enabled_extensions.mnd_headless = true;
    enabled_extensions.khr_convert_timespec_time = true;
    enabled_extensions.other.push(xdev_ext_name);

    if available_extensions.ext_hand_tracking {
        enabled_extensions.ext_hand_tracking = true;
    }

    let instance = entry
        .create_instance(
            &xr::ApplicationInfo {
                api_version: xr::Version::new(1, 0, 0),
                application_name: "motoc",
                application_version: 0,
                engine_name: "motoc",
                engine_version: 0,
            },
            &enabled_extensions,
            &[],
        )
        .map_err(|e| {
            Error::Context(
                "Failed to create OpenXR instance".into(),
                Box::new(e.into()),
            )
        })?;

    let instance_props = instance.properties().map_err(|e| {
        Error::Context(
            "Failed to query OpenXR instance properties".into(),
            Box::new(e.into()),
        )
    })?;
    log::info!(
        "Using OpenXR runtime: {} {}",
        instance_props.runtime_name,
        instance_props.runtime_version
    );

    let system = instance
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .map_err(|e| {
            Error::Context(
                "Failed to access OpenXR HMD system".into(),
                Box::new(e.into()),
            )
        })?;

    Ok((instance, system))
}

pub trait SpaceLocationConvert {
    fn into_transformd(self) -> Result<TransformD>;
}

impl SpaceLocationConvert for xr::SpaceLocation {
    fn into_transformd(self) -> Result<TransformD> {
        if !self.location_flags.contains(
            xr::SpaceLocationFlags::POSITION_TRACKED
                | xr::SpaceLocationFlags::POSITION_VALID
                | xr::SpaceLocationFlags::ORIENTATION_TRACKED
                | xr::SpaceLocationFlags::ORIENTATION_VALID,
        ) {
            return Err(Error::DeviceNotTracked);
        }

        Ok(self.pose.into())
    }
}

pub trait EffectiveSpaceVelocity {
    fn effective_angular(&self) -> Vector3<f32>;
    fn effective_linear(&self) -> Vector3<f32>;
}

impl EffectiveSpaceVelocity for xr::SpaceVelocity {
    fn effective_angular(&self) -> Vector3<f32> {
        if self
            .velocity_flags
            .intersects(xr::SpaceVelocityFlags::ANGULAR_VALID)
        {
            let v32: mint::Vector3<f32> = self.angular_velocity.into();
            v32.into()
        } else {
            Vector3::zeros()
        }
    }

    fn effective_linear(&self) -> Vector3<f32> {
        if self
            .velocity_flags
            .intersects(xr::SpaceVelocityFlags::LINEAR_VALID)
        {
            let v32: mint::Vector3<f32> = self.linear_velocity.into();
            v32.into()
        } else {
            Vector3::zeros()
        }
    }
}
