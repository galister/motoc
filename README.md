# Monado Tracking Origin Calibrator

This tool allows users to calibrate devices of different tracking origins (tracking technologies) to work together.

You will need Monado/WiVRn fresh from the main branch.

See [LVRA Wiki: WiVRn + Lighthouse Guide](https://lvra.gitlab.io/docs/fossvr/wivrn/#wivrn--lighthouse-driver) for more details about usage with WiVRn.

## Installing

Ensure Rust toolchain is up-to-date: `rustup update stable`

For Arch-based distros, AUR package `motoc-git` is available.

Other distros, install via cargo:

```bash
cargo install --git https://github.com/galister/motoc.git
```

## How to use

- With Monado/WiVRn running, use `motoc monitor` to identify the devices you'll be calibrating with.
- Note down the serial numbers (the part in double quotes) of the devices.
- Start the calibration:
  - `motoc calibrate --src "WiVRn HMD" --dst "LHR-ABCDE000"` (replace with your serials)
  - Add `--continue` if the tracker will stay attached to your headset.
- Move around so that the two devices move together in space.
  - If calibrating with a headset, hold the selected device firmly to the headset as you slowly walk around.
  - If the calibration did not succeed, it will retry automatically.

If the same tracker is attached to your headset the same way as last time, use `motoc continue` to re-use the last calibration.

## Optimization Tips for usage without a Head Tracker

### Inside-out tracking in general

Any headset that uses camera-based (SLAM) tracking may exibit issues when moving around the room; devices may line up perfectly in the center, but not so much when near to the edges.

Issues like this can be mitigated by adding objects with well-recognizable geometry to the room, such as a piece of furniture, a picture frame, etc. It is important that these do not move between sessions.

Also try to minimize reflective surfaces, such as mirrors or windows (use curtains, even translucent ones help tremendously!). Have a **well-lit room** when using `motoc`.

### Quest series

When used carefully in an optimized environment, Quest headsets are capable of holding calibration for weeks on end, regardless of the HMD sleeping or rebooting.

Quest series headsets have persistent stage tracking, and this is tied to (guardian) boundaries, so have boundaries enabled in the Quest's developer settings.

Having multiple boundaries drawn for the same room will result in the Quest picking one at random on startup, and the choice might not be the same one that was used for calibration.

**Clear your boundary history** via Quest settings > Tracking and **only ever keep one boundary**. When playing in a room where you don't plan on using `motoc`, either turn off the boundary via developer settings or use a stationary one.

Draw the boundary so that it lines up with some physical features of the playspace. This way, you can easily tell if the Quest is recognizing your playspace wrong. If it's off by a large amount, clear your boundary history and re-draw.

At the start of each session:
- If the Quest boundary was still lined up with your physical playspace: `motoc continue` - this will apply your previous calibration
- Otherwise (or if above fails): `motoc calibrate`

### Pico series

Pico headsets do not have persistent stage tracking, and so the headset will lose calibration whenever it goes to sleep.

A recommendation is to adjust or turn off the sleep timer: [r/PICO_VR: I managed to turn off the Pico 4's Sleep Mode](https://www.reddit.com/r/PICO_VR/comments/zmspi9/i_managed_to_turn_off_the_pico_4s_sleep_mode_by/)

The headset will not lose tracking if the screen is off, so the screen timer does not need to be adjusted, but place your Pico in a way that it can still see the environment. In particular, be careful not to cover too many of the cameras when taking the headset off, as it can momentarily lose tracking and re-initialize with a new stage.

With these tips, you are getting the most out of the Pico. However, it will still require doing a full `motoc calibrate` at the start of each session.


## Join the Linux VR Community

We are available on either:

- Discord: <https://discord.gg/gHwJ2vwSWV>
- Matrix Space: `#linux-vr-adventures:matrix.org`
