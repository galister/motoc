# Monado Tracking Origin Calibrator

This tool allows users to calibrate devices of different tracking origins (tracking technologies) to work together.

You will need version 25.0 or newer of Monado/WiVRn.

See [LVRA Wiki: WiVRn + Lighthouse Guide](https://lvra.gitlab.io/docs/fossvr/wivrn/#wivrn--lighthouse-driver) for more details about usage with WiVRn.

## Installing

For Arch-based distros, AUR package `motoc-git` is available.

Other distros, install via cargo:

```bash
cargo install --git https://github.com/galister/motoc.git
```

Or via Homebrew ([AtomicXR tap](https://codeberg.org/shiloh/homebrew-atomicxr)):
```bash
brew tap shiloh/atomicxr https://codeberg.org/shiloh/homebrew-atomicxr.git
brew install motoc
```

## How to use

- With Monado/WiVRn running, use `motoc monitor` to identify the devices you'll be calibrating with.
- Note down the serial numbers (the part in double quotes) of the devices.
- Start the calibration:
  - `sleep 5; motoc calibrate --src "WiVRn HMD" --dst "LHR-ABCDE000"` (replace with your serials)
  - Add `--continue` if the tracker will stay attached to your headset.
- Move around so that the two devices move together in space.
  - If calibrating with a headset, hold the selected device firmly to the headset as you slowly walk around.
  - Avoid erratic movements.
  - If the calibration did not succeed, it will retry automatically.

If the same tracker is attached to your headset the same way as last time, use `motoc continue` to re-use the last calibration.

## Calibration Tips

### Tips for high quality calibration

- The most important factor is that the devices are firmly held together, in a way that they move and rotate together perfectly.
- The devices must be held together in this way from the start to the end of the calibration. If you need time between starting the calibration and putting your devices together, add a `sleep` before `motoc calibrate`.
- Avoid erratic movements. Motion prediction is known to be different between LH driver and standalone devices. To avoid measurement errors from motion prediction being different, move the devices together in a slow and steady fashion.

### When and when not to use continuous mode?

Only use continuous mode if the 2 devices (tracker & HMD) are firmly attached.

Mounting a tracker to the HMD using velcro, rubber bands or other methods that allow for elastic deformation may be fine to use while at rest, but can induce overcorrections in high-movement scenarios like excercise or dancing.

If precision is needed during movement, mount the devices using a method that doesn't allow any wiggle in movement or rotation between the 2 devices, such as mounting brackets or glue.

### Calibrate with Standalone HMD or Standalone Controller?

The standalone headset will have some tracking wiggle as a result of inside-out-tracking. The controller is then tracked relative to the HMD. Since the controller itself also has some tracking wiggle, now we need'd to account for the tracking wiggle from both the HMD and the controller.

Therefore, I recommend calibrating with the HMD.

Disclaimer: This is all theory, in practice the difference in results may be miniscule.

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
