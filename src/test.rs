use nalgebra::Rotation3;

use crate::{
    common::{vec3, UNIT},
    transformd::TransformD,
};

fn euler_zxy(yaw: f64, pitch: f64, roll: f64) -> Rotation3<f64> {
    Rotation3::from_axis_angle(&UNIT.YU, yaw)
        * Rotation3::from_axis_angle(&UNIT.XU, pitch)
        * Rotation3::from_axis_angle(&UNIT.ZU, roll)
}

const EPS: f64 = 0.01;

fn mismatch(a: TransformD, b: TransformD) -> &'static str {
    if (a.origin - b.origin).norm_squared() > EPS {
        "position"
    } else if (a.basis * UNIT.X - b.basis * UNIT.X).norm_squared() > EPS {
        "unit_x"
    } else if (a.basis * UNIT.Y - b.basis * UNIT.Y).norm_squared() > EPS {
        "unit_y"
    } else if (a.basis * UNIT.Z - b.basis * UNIT.Z).norm_squared() > EPS {
        "unit_z"
    } else {
        ""
    }
}

#[test]
pub fn transform_hierarchy() {
    let expected_a = TransformD {
        origin: vec3(0.33, 0.5, 0.33),
        basis: euler_zxy((60f64).to_radians(), 0., 20f64.to_radians()),
    };

    let root_a = TransformD::default();
    let offset_a = TransformD::default();
    let object_a = TransformD {
        origin: vec3(0.33, 0.5, 0.33),
        basis: euler_zxy((60f64).to_radians(), 0., 20f64.to_radians()),
    };

    let pose_a = root_a * offset_a * object_a;

    assert_eq!(mismatch(pose_a, expected_a), "", "child");

    let root_b = TransformD {
        origin: vec3(-0.5, -1.0, 0.5),
        basis: euler_zxy((280f64).to_radians(), 0., 0.),
    };
    let offset_b = TransformD {
        origin: vec3(-0.406, 1.0, -0.579),
        basis: euler_zxy(80f64.to_radians(), 0.0, 0.0),
    };
    let object_b = TransformD {
        origin: vec3(0.46, 0.6, 0.405),
        basis: euler_zxy(60f64.to_radians(), 0., 15f64.to_radians()),
    };

    let pose_b = root_b * offset_b * object_b;
    let b_to_a = TransformD {
        origin: vec3(0., -0.1, -0.15),
        basis: euler_zxy(
            -0.1f64.to_radians(),
            -4.3f64.to_radians(),
            2.5f64.to_radians(),
        ),
    };

    assert_eq!(
        mismatch(pose_b * object_b.inverse() * offset_b.inverse(), root_b),
        "",
        "parent"
    );

    let expected_a = pose_b * b_to_a;

    assert_eq!(mismatch(expected_a, pose_a), "", "child2");

    assert_eq!(
        mismatch(pose_b.inverse() * pose_a, b_to_a),
        "",
        "inverse-child"
    );

    assert_eq!(
        mismatch(
            pose_a * b_to_a.inverse() * pose_b.inverse(),
            TransformD::default(),
        ),
        "",
        "offset"
    );
}
