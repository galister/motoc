use std::sync::LazyLock;

use libmonado_rs as mnd;
use nalgebra::{UnitVector3, Vector3};
use openxr as xr;

#[inline(always)]
#[allow(dead_code)]
pub const fn vec3<T>(x: T, y: T, z: T) -> Vector3<T> {
    Vector3::new(x, y, z)
}

#[allow(dead_code)]
#[allow(non_snake_case)]
pub struct UnitVectors {
    pub X: Vector3<f64>,
    pub Y: Vector3<f64>,
    pub Z: Vector3<f64>,
    pub NEG_X: Vector3<f64>,
    pub NEG_Y: Vector3<f64>,
    pub NEG_Z: Vector3<f64>,
    pub XU: UnitVector3<f64>,
    pub YU: UnitVector3<f64>,
    pub ZU: UnitVector3<f64>,
    pub NEG_XU: UnitVector3<f64>,
    pub NEG_YU: UnitVector3<f64>,
    pub NEG_ZU: UnitVector3<f64>,
}

#[allow(dead_code)]
pub static UNIT: LazyLock<UnitVectors> = LazyLock::new(|| UnitVectors {
    X: vec3(1., 0., 0.),
    Y: vec3(0., 1., 0.),
    Z: vec3(0., 0., 1.),
    NEG_X: vec3(-1., 0., 0.),
    NEG_Y: vec3(0., -1., 0.),
    NEG_Z: vec3(0., 0., -1.),
    XU: UnitVector3::new_unchecked(vec3(1., 0., 0.)),
    YU: UnitVector3::new_unchecked(vec3(0., 1., 0.)),
    ZU: UnitVector3::new_unchecked(vec3(0., 0., 1.)),
    NEG_XU: UnitVector3::new_unchecked(vec3(-1., 0., 0.)),
    NEG_YU: UnitVector3::new_unchecked(vec3(0., -1., 0.)),
    NEG_ZU: UnitVector3::new_unchecked(vec3(0., 0., -1.)),
});

#[allow(dead_code)]
pub struct Device {
    pub name: String,
    pub serial: String,
    pub index: u32,
    pub tracking_origin: u32,
    pub space: xr::Space,
}

pub struct CalibratorData<'a> {
    pub tracking_origins: Vec<mnd::TrackingOrigin<'a>>,
    pub devices: Vec<Device>,
    pub stage: xr::Space,
    pub now: xr::Time,
}

impl<'a> CalibratorData<'a> {
    pub fn find_device(&self, serial: &str) -> Option<usize> {
        self.devices.iter().position(|d| d.serial == *serial)
    }

    pub fn get_device_origin(&self, device: usize) -> anyhow::Result<mnd::TrackingOrigin<'a>> {
        let Some(device) = self.devices.get(device) else {
            anyhow::bail!("no such device: {}", device);
        };
        let Some(origin) = self.tracking_origins.get(device.tracking_origin as usize) else {
            anyhow::bail!("no such tracking origin: {}", device.tracking_origin);
        };

        Ok(origin.clone())
    }
}