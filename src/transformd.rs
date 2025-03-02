use std::fmt::Display;

use libmonado as mnd;
use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector3};
use openxr as xr;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct TransformD {
    pub basis: Rotation3<f64>,
    pub origin: Vector3<f64>,
}

impl std::ops::Mul for TransformD {
    type Output = Self;
    fn mul(self, rhs: TransformD) -> Self::Output {
        Self {
            origin: self.origin + self.basis * rhs.origin,
            basis: self.basis * rhs.basis,
        }
    }
}

impl TransformD {
    pub fn lerp(self, other: Self, s: f64) -> Self {
        Self {
            origin: self.origin.lerp(&other.origin, s),
            basis: self
                .basis
                .try_slerp(&other.basis, s, f32::EPSILON as f64)
                .unwrap_or(self.basis),
        }
    }

    pub fn inverse(self) -> Self {
        let transpose = self.basis.transpose();
        Self {
            origin: transpose * self.origin.scale(-1f64),
            basis: transpose,
        }
    }

    pub fn direction(self) -> Self {
        Self {
            origin: Vector3::default(),
            basis: self.basis,
        }
    }
}

impl From<TransformD> for mnd::Pose {
    fn from(value: TransformD) -> Self {
        let q = UnitQuaternion::from_rotation_matrix(&value.basis);
        mnd::Pose {
            orientation: q.cast().into(),
            position: value.origin.cast().into(),
        }
    }
}

impl From<mnd::Pose> for TransformD {
    fn from(value: mnd::Pose) -> Self {
        let v32: Vector3<f32> = value.position.into();
        let q32: Quaternion<f32> = value.orientation.into();
        let uq: UnitQuaternion<f64> = UnitQuaternion::from_quaternion(q32.cast());
        TransformD {
            basis: uq.to_rotation_matrix(),
            origin: v32.cast(),
        }
    }
}

impl From<xr::Posef> for TransformD {
    fn from(value: xr::Posef) -> Self {
        let v32: mint::Vector3<f32> = value.position.into();
        let q32: mint::Quaternion<f32> = value.orientation.into();
        let v32: Vector3<f32> = v32.into();
        let q32: Quaternion<f32> = q32.into();
        let uq: UnitQuaternion<f64> = UnitQuaternion::from_quaternion(q32.cast());
        TransformD {
            basis: uq.to_rotation_matrix(),
            origin: v32.cast(),
        }
    }
}

impl Default for TransformD {
    fn default() -> Self {
        TransformD {
            basis: Rotation3::identity(),
            origin: Vector3::zeros(),
        }
    }
}

impl Display for TransformD {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (roll, pitch, yaw) = self.basis.euler_angles();
        write!(
            f,
            "(X: {:.2}, Y: {:.2}, Z: {:.2} | Yaw: {:.2}, Pitch: {:.2}, Roll: {:.2})",
            self.origin.x,
            self.origin.y,
            self.origin.z,
            yaw.to_degrees(),
            pitch.to_degrees(),
            roll.to_degrees()
        )
    }
}
