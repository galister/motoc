use std::f32::consts::PI;

use colored::{Color, Colorize};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use openxr::{SpaceLocationFlags, SpaceVelocityFlags};

use super::{Calibrator, StepResult};

pub struct Monitor {}

impl Monitor {
    pub fn new() -> Self {
        Self {}
    }
}

impl Calibrator for Monitor {
    fn init(
        &mut self,
        _: &mut crate::common::CalibratorData,
        _: &mut indicatif::MultiProgress,
    ) -> anyhow::Result<super::StepResult> {
        Ok(StepResult::Continue)
    }

    fn step(
        &mut self,
        data: &mut crate::common::CalibratorData,
    ) -> anyhow::Result<super::StepResult> {
        print!("{esc}[2J{esc}[1;1H", esc = 27 as char);

        for to in data.tracking_origins.iter() {
            println!("[{}] {}", to.id, to.name);
            let pose = to.get_offset()?;
            let (roll, pitch, yaw) =
                UnitQuaternion::from_quaternion(Quaternion::from(pose.orientation)).euler_angles();
            println!(
                " │ POS: (X: {:.2}, Y: {:.2}, Z: {:.2})",
                pose.position.x, pose.position.y, pose.position.z
            );
            println!(" │ ROT: (Y: {:.2}, P: {:.2}, R: {:.2})", yaw, pitch, roll);

            let to_devs = data
                .devices
                .iter()
                .filter(|d| d.tracking_origin == to.id)
                .collect::<Vec<_>>();

            let last = to_devs.len() - 1;

            for (i, d) in to_devs.iter().enumerate() {
                let branch = if last == i { '└' } else { '├' };
                let serial = &d.serial;
                print!(" {}── [{}] \"{}\"", branch, d.index, serial);
                if !d.name.is_empty() && d.name != *serial {
                    print!(" ({})", d.name);
                }

                let branch = if last == i { ' ' } else { '│' };
                println!();

                let (loc, vel) = d.space.relate(&data.stage, data.now)?;

                print!(" {}    ├── Position: ", branch);

                if !loc
                    .location_flags
                    .intersects(SpaceLocationFlags::POSITION_VALID)
                {
                    println!("Invalid");
                } else {
                    let color = if loc
                        .location_flags
                        .intersects(SpaceLocationFlags::POSITION_TRACKED)
                    {
                        Color::Green
                    } else {
                        Color::Red
                    };
                    println!(
                        "{}",
                        format!(
                            "X: {:.2}, Y: {:.2}, Z: {:.2}",
                            loc.pose.position.x, loc.pose.position.y, loc.pose.position.z
                        )
                        .color(color)
                    );
                }

                print!(" {}    ├── Rotation: ", branch);

                if !loc
                    .location_flags
                    .intersects(SpaceLocationFlags::ORIENTATION_VALID)
                {
                    println!("Invalid");
                } else {
                    let color = if loc
                        .location_flags
                        .intersects(SpaceLocationFlags::ORIENTATION_TRACKED)
                    {
                        Color::Green
                    } else {
                        Color::Red
                    };

                    let q32: mint::Quaternion<f32> = loc.pose.orientation.into();
                    let q: UnitQuaternion<f32> = UnitQuaternion::from_quaternion(q32.into());
                    let (roll, pitch, yaw) = q.euler_angles();

                    println!(
                        "{}",
                        format!("Y: {:.2}, P: {:.2}, R: {:.2}", yaw, pitch, roll).color(color)
                    );
                }

                print!(" {}    ├── Speed: ", branch);

                if !vel
                    .velocity_flags
                    .intersects(SpaceVelocityFlags::LINEAR_VALID)
                {
                    println!("Invalid");
                } else {
                    let v32: mint::Vector3<f32> = vel.linear_velocity.into();
                    let linear: Vector3<f32> = v32.into();
                    let speed = linear.norm();
                    let ticks = (speed / 3.0 * 20.0).clamp(0., 20.) as usize;
                    println!(
                        "[{}{}] {:.2} m/s",
                        "=".repeat(ticks).color(Color::Red),
                        " ".repeat(20 - ticks),
                        speed
                    );
                }

                print!(" {}    └── Spin:  ", branch);

                if !vel
                    .velocity_flags
                    .intersects(SpaceVelocityFlags::ANGULAR_VALID)
                {
                    println!("Invalid");
                } else {
                    let v32: mint::Vector3<f32> = vel.angular_velocity.into();
                    let angular: Vector3<f32> = v32.into();
                    let spin = angular.norm();
                    let ticks = (spin / PI * 5.0).clamp(0., 20.) as usize;
                    println!(
                        "[{}{}] {:.2} rad/s",
                        "=".repeat(ticks).color(Color::Red),
                        " ".repeat(20 - ticks),
                        spin
                    );
                }
            }
        }

        Ok(StepResult::Continue)
    }
}
