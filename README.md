# Monado Tracking Origin Calibrator

This tool allows users to calibrate devices of different tracking origins (tracking technologies) to work together.

You will need Monado/WiVRn fresh from the main branch.

## Building

Ensure your rust toolchain is up to date: `rustup update stable`

```bash
cargo build --release
```

## How to use

- With Monado/WiVRn running, use `motoc monitor` to identify the devices you'll be calibrating with.
- Note down the serial numbers (the part in double quotes) of the devices.
- Start the calibration:
  - `motoc calibrate --src "WiVRn HMD" --dst "LHR-ABCDE000"` (replace with your serials)
  - Add `--continue` if the tracker will stay attached to your headset.
- Move around so that the two devices move together in space.
  - If the calibration did not succeed, it will retry automatically.

If the same tracker is attached to your headset the same way as last time, use `motoc continue` to re-use the last calibration.

## Join the Linux VR Community

We are available on either:

- Discord: <https://discord.gg/gHwJ2vwSWV>
- Matrix Space: `#linux-vr-adventures:matrix.org`
