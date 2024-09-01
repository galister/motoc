use nalgebra::Vector3;
use openxr::{self as xr};

use crate::{mndx, transformd::TransformD};

pub(crate) fn xr_init() -> anyhow::Result<(xr::Instance, xr::SystemId)> {
    let entry = xr::Entry::linked();

    let Ok(available_extensions) = entry.enumerate_extensions() else {
        anyhow::bail!("Failed to enumerate OpenXR extensions.");
    };

    anyhow::ensure!(
        available_extensions.mnd_headless,
        "Missing MND_headless extension."
    );

    let xdev_ext_name = mndx::XDEV_SPACE_EXTENSION_NAME.into();

    anyhow::ensure!(
        available_extensions.other.contains(&xdev_ext_name),
        "Missing MNDX_xdev_space extension."
    );

    let mut enabled_extensions = xr::ExtensionSet::default();
    enabled_extensions.mnd_headless = true;
    enabled_extensions.khr_convert_timespec_time = true;
    enabled_extensions.other.push(xdev_ext_name);

    let Ok(instance) = entry.create_instance(
        &xr::ApplicationInfo {
            api_version: xr::Version::new(1, 0, 0),
            application_name: "motoc",
            application_version: 0,
            engine_name: "motoc",
            engine_version: 0,
        },
        &enabled_extensions,
        &[],
    ) else {
        anyhow::bail!("Failed to create OpenXR instance.");
    };

    let Ok(instance_props) = instance.properties() else {
        anyhow::bail!("Failed to query OpenXR instance properties.");
    };
    log::info!(
        "Using OpenXR runtime: {} {}",
        instance_props.runtime_name,
        instance_props.runtime_version
    );

    let Ok(system) = instance.system(xr::FormFactor::HEAD_MOUNTED_DISPLAY) else {
        anyhow::bail!("Failed to access OpenXR HMD system.");
    };

    Ok((instance, system))
}

pub trait SpaceLocationConvert {
    fn into_transformd(self) -> anyhow::Result<TransformD>;
}

impl SpaceLocationConvert for xr::SpaceLocation {
    fn into_transformd(self) -> anyhow::Result<TransformD> {
        if !self.location_flags.contains(
            xr::SpaceLocationFlags::POSITION_TRACKED
                | xr::SpaceLocationFlags::POSITION_VALID
                | xr::SpaceLocationFlags::ORIENTATION_TRACKED
                | xr::SpaceLocationFlags::ORIENTATION_VALID,
        ) {
            anyhow::bail!("device not tracked");
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
