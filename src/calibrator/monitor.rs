use std::f32::consts::PI;

use colored::{Color, Colorize};
use libmonado as mnd;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use openxr::{SpaceLocationFlags, SpaceVelocityFlags};

use super::{Calibrator, StepResult};

const TICKER_SIZE: usize = 10;

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
        let stage = data
            .monado
            .get_reference_space_offset(mnd::ReferenceSpaceType::Stage)?;
        let (roll, pitch, yaw) =
            UnitQuaternion::from_quaternion(Quaternion::from(stage.orientation)).euler_angles();
        println!("{}", "[STAGE] Reference".bright_blue());
        let pos = format!(
            "X: {:.2}, Y: {:.2}, Z: {:.2}",
            stage.position.x, stage.position.y, stage.position.z
        );
        let space = " ".repeat(30 - pos.len().min(35));
        println!("       {pos} {space} Yaw: {yaw:.2}, Pitch: {pitch:.2}, Roll: {roll:.2}");

        println!("\n{}", "[LOCAL] Reference".bright_blue());
        let local = data
            .monado
            .get_reference_space_offset(mnd::ReferenceSpaceType::Local)?;
        let (roll, pitch, yaw) =
            UnitQuaternion::from_quaternion(Quaternion::from(local.orientation)).euler_angles();
        let pos = format!(
            "X: {:.2}, Y: {:.2}, Z: {:.2}",
            local.position.x, local.position.y, local.position.z
        );
        let space = " ".repeat(30 - pos.len().min(35));
        println!("       {pos} {space} Yaw: {yaw:.2}, Pitch: {pitch:.2}, Roll: {roll:.2}");

        for to in data.tracking_origins.iter() {
            println!("\n{}", format!("[{}] {}", to.id, to.name).bright_blue());
            let pose = to.get_offset()?;
            let (roll, pitch, yaw) =
                UnitQuaternion::from_quaternion(Quaternion::from(pose.orientation)).euler_angles();
            let pos = format!(
                "X: {:.2}, Y: {:.2}, Z: {:.2}",
                pose.position.x, pose.position.y, pose.position.z
            );
            let space = " ".repeat(30 - pos.len().min(35));
            println!(" │     {pos} {space} Yaw: {yaw:.2}, Pitch: {pitch:.2}, Roll: {roll:.2}");

            let to_devs = data
                .devices
                .iter()
                .filter(|d| d.tracking_origin == to.id)
                .collect::<Vec<_>>();

            let last = to_devs.len() - 1;

            for (i, d) in to_devs.iter().enumerate() {
                let branch = if last == i { '└' } else { '├' };
                let branch2 = if last == i { ' ' } else { '│' };
                let serial = &d.serial;
                println!(" │");
                print!(
                    " {}── {}",
                    branch,
                    format!("[{}] \"{}\"", d.index, serial).bright_yellow()
                );
                if !d.inner.name.is_empty() && d.inner.name != *serial {
                    print!("{}", format!(" ({})", d.inner.name).bright_yellow());
                }

                if let Ok(battery) = d.inner.battery_status() {
                    if battery.present {
                        let symbol = if battery.charging { '⚡' } else { '🔋' };
                        print!(
                            " {}",
                            format!("{}{:.0}%", symbol, battery.charge * 100.0).color(
                                if battery.charging {
                                    Color::BrightBlue
                                } else if battery.charge > 0.4 {
                                    Color::BrightGreen
                                } else if battery.charge > 0.2 {
                                    Color::Yellow
                                } else {
                                    Color::BrightRed
                                }
                            )
                        );
                    }
                }

                println!();

                let (loc, vel) = d.space.relate(&data.stage, data.now)?;

                let pos = format!(
                    "X: {:.2}, Y: {:.2}, Z: {:.2}",
                    loc.pose.position.x, loc.pose.position.y, loc.pose.position.z
                )
                .color(
                    if loc.location_flags.intersects(
                        SpaceLocationFlags::POSITION_VALID | SpaceLocationFlags::POSITION_TRACKED,
                    ) {
                        Color::White
                    } else {
                        Color::BrightBlack
                    },
                );

                let space = " ".repeat(30 - pos.len().min(35));

                let rot = {
                    let q32: mint::Quaternion<f32> = loc.pose.orientation.into();
                    let q: UnitQuaternion<f32> = UnitQuaternion::from_quaternion(q32.into());
                    let (roll, pitch, yaw) = q.euler_angles();

                    format!("Yaw: {yaw:.2}, Pitch: {pitch:.2}, Roll: {roll:.2}").color(
                        if loc.location_flags.intersects(
                            SpaceLocationFlags::ORIENTATION_VALID
                                | SpaceLocationFlags::ORIENTATION_TRACKED,
                        ) {
                            Color::White
                        } else {
                            Color::BrightBlack
                        },
                    )
                };

                println!(" {branch2}     {pos} {space} {rot}");

                let speed = {
                    let v32: mint::Vector3<f32> = vel.linear_velocity.into();
                    let linear: Vector3<f32> = v32.into();
                    let speed = linear.norm();
                    let ticks =
                        (speed * (TICKER_SIZE as f32)).clamp(0., TICKER_SIZE as f32) as usize;
                    format!(
                        "Speed: [{}{}] {:.2} m/s",
                        "=".repeat(ticks).color(Color::BrightGreen),
                        " ".repeat(TICKER_SIZE - ticks),
                        speed
                    )
                    .color(
                        if vel
                            .velocity_flags
                            .intersects(SpaceVelocityFlags::LINEAR_VALID)
                        {
                            Color::White
                        } else {
                            Color::BrightBlack
                        },
                    )
                };

                let spin = {
                    let v32: mint::Vector3<f32> = vel.angular_velocity.into();
                    let angular: Vector3<f32> = v32.into();
                    let spin = angular.norm();
                    let ticks = (spin / PI * 2.0).clamp(0., TICKER_SIZE as f32) as usize;
                    format!(
                        "Spin: [{}{}] {:.2} rad/s",
                        "=".repeat(ticks).color(Color::BrightGreen),
                        " ".repeat(TICKER_SIZE - ticks),
                        spin
                    )
                    .color(
                        if vel
                            .velocity_flags
                            .intersects(SpaceVelocityFlags::ANGULAR_VALID)
                        {
                            Color::White
                        } else {
                            Color::BrightBlack
                        },
                    )
                };

                println!(" {branch2}     {speed}    {spin}");
            }
        }

        Ok(StepResult::Continue)
    }
}
