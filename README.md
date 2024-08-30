# Monado Tracking Origin Calibrator

This tool allows users to calibrate devices of different tracking origins (tracking technologies) to work together.

## Building

Ensure your rust toolchain is up to date: `rustup update stable`

Required Monado patch: [MR#2313 on Monado GitLab](https://gitlab.freedesktop.org/monado/monado/-/merge_requests/2313)

```bash
cargo build --release
```

## How to use

- Once Monado/WiVRn is running, use `motoc show` to show available tracking origins and their devices.
- See `motoc calibrate --help` about how to use the calibrate command.
  - You will need to provide the serial number of the source and target devices.

## Join the Linux VR Community

We are available on either:

- Discord: <https://discord.gg/gHwJ2vwSWV>
- Matrix Space: `#linux-vr-adventures:matrix.org`
