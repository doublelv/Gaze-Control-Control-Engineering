# Gaze-Control-Control-Engineering
Public repo for Fontys Master System Design, Control Engineering class, Group 1.

A 3-axis eye-tracking rig: a camera carriage driven by a DC motor on a belt follows the user's
head, two servo-tilted IR LEDs light the eyes, and MATLAB tracks the pupils, the head and
(after calibration) the gaze point on the screen.

**Status:** complete software (MATLAB + STM32 firmware). All offline tests pass (Octave + gcc,
see CI). **Not yet run on the hardware.** Values marked `[PLACEHOLDER]` / `[CHECK]` in
`matlab/config/params.m` and `firmware/App/config.h` must be measured on the rig.

## Quick start (no hardware)
In MATLAB, Current Folder = `matlab/scripts`, then run:
- `run_eye_demo`: eye + head tracking on a synthetic face (set `SOURCE = 'webcam'` for the OV9281)
- `run_rig`: complete rig in virtual mode (simulated Nucleo, motor and camera)
- `run_calibration`: gaze calibration with a virtual user

## Repository
| Folder | Content |
|---|---|
| `matlab/` | Camera, eye detection, head tracking, gaze, control, communication, visualisation, scripts, tests. Start with `matlab/README.md` |
| `firmware/` | NUCLEO-H753ZI I/O bridge in C (`App/`), host tests, CubeMX setup guide |
| `interface/` | MATLAB <-> STM32 protocol v1 + shared test vectors |
| `docs/` | Requirements, architecture, control design, hardware, decisions |
| `tools/` | `run_tests.sh` (all offline tests), test-vector generator |
| `.github/workflows/` | CI: firmware host tests + MATLAB tests in Octave |

## Hardware
NUCLEO-H753ZI, MDD10A + FIT0186 geared motor with encoder, GT2 belt on 2020 V-slot,
2 x SER0006 servos, 4 x 850 nm IR LEDs, OV9281 USB camera (1.65 mm lens), Mean Well 12 V.
Details and wiring: `docs/hardware/hardware.md`.

## Tests
`tools/run_tests.sh`, or in MATLAB: `cd matlab/tests; run_all_tests`, and `make -C firmware/tests`.

Read `AGENTS.md` before contributing. Do not commit recordings of eyes or calibration files
(`matlab/data/local/`, `matlab/config/local/` are git-ignored).
