use std::{collections::HashMap, thread, time::Duration};

use calibrator::{Calibrator, SampledMethod, StepResult};
use clap::{Command, FromArgMatches, Subcommand};
use common::{CalibratorData, Device};
use env_logger::Env;
use indicatif::MultiProgress;
use indicatif_log_bridge::LogWrapper;
use libmonado_rs as mnd;
use nalgebra::{Quaternion, UnitQuaternion};
use openxr as xr;
use transformd::TransformD;

mod calibrator;
mod common;
mod helpers_xr;
mod mndx;
mod transformd;

#[cfg(test)]
mod test;

fn main() {
    let log = env_logger::Builder::from_env(Env::default().default_filter_or("info")).build();
    let status = MultiProgress::new();
    LogWrapper::new(status.clone(), log).try_init().unwrap();

    let main = Command::new(env!("CARGO_PKG_NAME"))
        .about("Monado Tracking Origin Calibrator")
        .version(env!("CARGO_PKG_VERSION"));

    let mut args = Subcommands::augment_subcommands(main.clone());
    let Ok(subcommands) = Subcommands::from_arg_matches(&args.clone().get_matches()) else {
        let _ = args.print_long_help();
        return;
    };

    let Ok(monado) = mnd::Monado::auto_connect() else {
        return;
    };

    let required_libmonado_version = mnd::Version::new(1, 3, 1);
    let libmonado_version = monado.get_api_version();
    if libmonado_version < required_libmonado_version {
        log::error!("Your libmonado API version is not supported.");
        log::error!("Please update your Monado/WiVRn installation.");
        log::error!("Required: API {} or later", required_libmonado_version);
        log::error!("Current: API {}", libmonado_version);
        return;
    }

    match handle_non_xr_subcommands(&subcommands, &monado) {
        Ok(true) => return,
        Ok(false) => {}
        Err(e) => {
            log::error!("{:?}", e);
            return;
        }
    }

    if let Err(e) = xr_loop(subcommands, monado, status) {
        log::error!("{:?}", e);
        // return;
    }
}

fn handle_non_xr_subcommands(
    subcommands: &Subcommands,
    monado: &mnd::Monado,
) -> anyhow::Result<bool> {
    match subcommands {
        Subcommands::Show => {
            let mut devs = vec![];
            let mut dev_tos = vec![];

            for d in monado.devices()?.into_iter() {
                if !d.get_info_bool(mnd::MndProperty::PropertySupportsPositionBool)? {
                    continue;
                }
                dev_tos.push(d.get_info_u32(mnd::MndProperty::PropertyTrackingOriginU32)?);
                devs.push(d);
            }

            for to in monado.tracking_origins()?.into_iter() {
                println!("[{}] {}", to.id, to.name);
                let pose = to.get_offset()?;
                let (roll, pitch, yaw) =
                    UnitQuaternion::from_quaternion(Quaternion::from(pose.orientation))
                        .euler_angles();
                println!(
                    " │ POS: (X: {:.2}, Y: {:.2}, Z: {:.2})",
                    pose.position.x, pose.position.y, pose.position.z
                );
                println!(" │ ROT: (Y: {:.2}, P: {:.2}, R: {:.2})", yaw, pitch, roll);

                let to_devs = devs
                    .iter()
                    .enumerate()
                    .filter_map(|(i, d)| if dev_tos[i] == to.id { Some(d) } else { None })
                    .collect::<Vec<_>>();

                let last = to_devs.len() - 1;

                for (i, d) in to_devs.iter().enumerate() {
                    let branch = if last == i { '└' } else { '├' };
                    let serial = d.serial()?;
                    print!(" {}── [{}] \"{}\"", branch, d.index, serial);
                    if !d.name.is_empty() && d.name != serial {
                        print!(" ({})", d.name);
                    }
                    println!();
                }
            }
            Ok(true)
        }
        Subcommands::Reset { id } => {
            for to in monado.tracking_origins()?.into_iter() {
                if to.id != *id {
                    continue;
                }
                match to.set_offset(TransformD::default().into()) {
                    Ok(_) => println!("{} has been reset.", to.name),
                    Err(e) => println!("Could not reset due to libmonado error: {:?}", e),
                }
                break;
            }
            Ok(true)
        }
        _ => Ok(false),
    }
}

fn xr_loop(
    subcommands: Subcommands,
    monado: mnd::Monado,
    mut status: MultiProgress,
) -> anyhow::Result<()> {
    let (instance, system) = helpers_xr::xr_init()?;

    let actions = instance.create_action_set("motoc", "MoToC", 0)?;

    let (session, _, _) = unsafe {
        instance
            .create_session::<xr::headless::Headless>(system, &xr::headless::SessionCreateInfo {})?
    };

    session.attach_action_sets(&[&actions])?;

    let mndx = mndx::Mndx::new(&instance)?;

    log::info!("LibMonado API version {}", monado.get_api_version());

    let mut events = xr::EventDataBuffer::new();
    let mut session_running = false;

    let mut calibrator_data = None;
    let mut calibrator: Option<Box<dyn Calibrator>> = None;

    'main_loop: loop {
        'event_loop: while let Some(event) = instance.poll_event(&mut events)? {
            use xr::Event::*;
            match event {
                SessionStateChanged(e) => match e.state() {
                    xr::SessionState::READY => {
                        session.begin(xr::ViewConfigurationType::PRIMARY_STEREO)?;
                        session_running = true;
                        log::info!("XrSession started.");

                        if calibrator_data.is_some() {
                            continue 'event_loop;
                        }

                        let mut data = load_calibrator_data(&session, &mndx, &monado)?;

                        #[allow(clippy::single_match)] // TODO: fix offset mode
                        match subcommands {
                            /*
                            Subcommands::Offset {
                                ref src,
                                ref dst,
                                yaw,
                                pitch,
                                roll,
                                x,
                                y,
                                z,
                                lerp,
                            } => {
                                let Some(src_dev) = data.find_device(src) else {
                                    log::error!("src: no such device: {}", &src);
                                    break 'main_loop;
                                };
                                let Some(dst_dev) = data.find_device(dst) else {
                                    log::error!("dst: no such device: {}", &dst);
                                    break 'main_loop;
                                };

                                if data.devices[src_dev].tracking_origin
                                    == data.devices[dst_dev].tracking_origin
                                {
                                    log::error!("both devices are in the same tracking origin");
                                    break 'main_loop;
                                }

                                calibrator = Some(Box::new({
                                    let mut c = OffsetMethod::new(
                                        src_dev,
                                        dst_dev,
                                        Vector3::new(
                                            pitch.unwrap_or(0.0),
                                            yaw.unwrap_or(0.0),
                                            roll.unwrap_or(0.0),
                                        ),
                                        Vector3::new(
                                            x.unwrap_or(0.0),
                                            y.unwrap_or(0.0),
                                            z.unwrap_or(0.0),
                                        ),
                                        lerp.unwrap_or(0.05),
                                    );
                                    c.init(&mut data, &mut status)?;
                                    c
                                }));
                            } */
                            Subcommands::Calibrate {
                                ref src,
                                ref dst,
                                r#continue: maintain,
                                samples,
                            } => {
                                let Some(src_dev) = data.find_device(src) else {
                                    log::error!("src: no such device: {}", &src);
                                    break 'main_loop;
                                };
                                let Some(dst_dev) = data.find_device(dst) else {
                                    log::error!("dst: no such device: {}", &dst);
                                    break 'main_loop;
                                };

                                if data.devices[src_dev].tracking_origin
                                    == data.devices[dst_dev].tracking_origin
                                {
                                    log::error!("both devices are in the same tracking origin");
                                    break 'main_loop;
                                }

                                calibrator = Some(Box::new({
                                    let mut c = SampledMethod::new(
                                        src_dev,
                                        dst_dev,
                                        maintain,
                                        samples.unwrap_or(500),
                                    );
                                    c.init(&mut data, &mut status)?;
                                    c
                                }));
                            }
                            _ => {}
                        }
                        calibrator_data = Some(data);
                    }
                    xr::SessionState::STOPPING => {
                        session.end()?;
                        session_running = false;
                        log::info!("XrSession stopped.")
                    }
                    xr::SessionState::EXITING | xr::SessionState::LOSS_PENDING => {
                        anyhow::bail!("XR session exiting");
                    }
                    _ => {}
                },
                InstanceLossPending(_) => {
                    anyhow::bail!("XR instance loss pending");
                }
                EventsLost(e) => {
                    log::warn!("lost {} events", e.lost_event_count());
                }
                _ => {}
            }
        }

        if !session_running {
            thread::sleep(Duration::from_millis(40));
            continue 'main_loop;
        }

        session.sync_actions(&[(&actions).into()])?;

        if let (Some(data), Some(cal)) = (calibrator_data.as_mut(), calibrator.as_mut()) {
            data.now = instance.now()?;
            match cal.step(data)? {
                StepResult::End => {
                    status.clear()?;
                    log::info!("Our work here is done! ✅");
                    //TODO: have some interactibility
                    // calibrator = None;
                    break 'main_loop;
                }
                StepResult::Replace(mut new_calibrator) => {
                    status.clear()?;
                    new_calibrator.init(data, &mut status)?;
                    calibrator = Some(new_calibrator);
                }
                StepResult::Continue => {}
            }
        }

        thread::sleep(Duration::from_millis(40));
    }

    Ok(())
}

fn load_calibrator_data<'a, G>(
    session: &xr::Session<G>,
    mndx: &mndx::Mndx,
    monado: &'a mnd::Monado,
) -> anyhow::Result<CalibratorData<'a>> {
    let mut devices = vec![];
    let mut tracking_origins = vec![];

    let mut serial_space = HashMap::new();

    let xdev_list = mndx.create_list(session)?;
    for xdev in xdev_list.enumerate_xdevs()?.into_iter() {
        if !xdev.can_create_space() {
            continue;
        }
        serial_space.insert(
            xdev.serial().to_string(),
            xdev.create_space(session.clone())?,
        );
    }

    for dev in monado.devices()?.into_iter() {
        let serial = dev.serial()?;
        let Some(space) = serial_space.remove(serial.as_str()) else {
            continue;
        };

        devices.push(Device {
            tracking_origin: dev.get_info_u32(mnd::MndProperty::PropertyTrackingOriginU32)?,
            serial,
            space,
            name: dev.name,
            index: dev.index,
        });
    }

    for to in monado.tracking_origins()?.into_iter() {
        tracking_origins.push(to);
    }

    Ok(CalibratorData {
        devices,
        tracking_origins,
        stage: session
            .create_reference_space(xr::ReferenceSpaceType::STAGE, xr::Posef::IDENTITY)?,
        now: session.instance().now()?,
    })
}

#[derive(clap::Parser, Debug)]
enum Subcommands {
    /// Show available tracking origings and their devices
    Show,

    /*
    /// Maintain a static offset between two devices
    Offset {
        /// the source device (usu. HMD)
        #[arg(long, value_name = "SERIAL_NUMBER")]
        src: String,

        /// the destination device (usu. tracker)
        #[arg(long, value_name = "SERIAL_NUMBER")]
        dst: String,

        /// rotation offset from dst to src device in DEGREES
        #[arg(long)]
        yaw: Option<f64>,

        /// rotation offset from dst to src device in DEGREES
        #[arg(long)]
        pitch: Option<f64>,

        /// rotation offset from dst to src device in DEGREES
        #[arg(long)]
        roll: Option<f64>,

        /// position offset from dst to src device in METERS
        #[arg(long)]
        x: Option<f64>,

        /// position offset from dst to src device in METERS
        #[arg(long)]
        y: Option<f64>,

        /// position offset from dst to src device in METERS
        #[arg(long)]
        z: Option<f64>,

        /// interpolation factor, lower is smoother. range (0, 1]
        #[arg(long, value_name = "FACTOR")]
        lerp: Option<f64>,
    },
    */
    /// Calibrate by sampling two devices that move together over time
    Calibrate {
        /// the source device (usu. HMD)
        #[arg(long, value_name = "SERIAL_NUMBER")]
        src: String,

        /// the destination device (usu. tracker)
        #[arg(long, value_name = "SERIAL_NUMBER")]
        dst: String,

        /// continue maintaining offset after calibration. enable if the devices are firmly attached
        #[arg(long)]
        r#continue: bool,

        /// number of samples to use for initial calibration. default: 500
        #[arg(long)]
        samples: Option<u32>,
    },
    /// Reset the offset for the given tracking origin.
    Reset {
        /// tracking origin ID from `motoc show`
        #[arg(value_name = "ORIGIN")]
        id: u32,
    },
}
