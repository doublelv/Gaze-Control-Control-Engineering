# Gaze-Control-Control-Engineering
Public repo for Fontys Master System Design, Control Engineering class, Group 1.

A 3-axis eye-tracking rig: a camera carriage driven by a DC motor on a belt follows the user's
head, two pan servos on the frame turn IR LEDs toward the eyes, and MATLAB tracks the pupils, the head and
(after calibration) the gaze point on the screen.

**Status (2026-11-06):**
- Runs on the rig: firmware, link, stall homing at both ends, safety checks, open-loop
  identification (`sysid`) and **face tracking** (`run_rig`, `TASK='track_camera'`, `P.trk.source='face'`).
- IR LEDs mounted (2026-11-06, on both servos and next to the camera); `P.trk.source='both'`
  (eyes inside the face box, else the face) is the default but not yet run on the rig.
- Not yet done on the rig: eye/pupil tracking and gaze calibration with the
  LEDs, servo 1 (not wired), servo aiming, the step test (`TASK='step'`), and the retuned carriage
  controller in `params.m` (zeta 1.5, b = 0.6; so far checked in simulation only).
- All offline tests pass (MATLAB/Octave + gcc, see CI). Values marked `[PLACEHOLDER]` / `[CHECK]` in
  `matlab/config/params.m` and `firmware/App/config.h` are still unmeasured.

## Quick start
In MATLAB, Current Folder = `matlab/scripts`.
- No hardware: `run_rig` with `MODE='virtual'` (simulated Nucleo, motor and camera), `run_eye_demo`
  with the synthetic face, `run_all_tests` in `matlab/tests`.
- On the rig: plug in the Nucleo USB first, then the 12 V supply, then `run_rig` with `MODE='serial'`.
  Which files are used on the rig and which only in simulation/tests: `matlab/README.md` (status legend).

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
NUCLEO-H753ZI, Cytron MD10C + FIT0186 geared motor with encoder, GT2 belt on 2020 V-slot,
2 x SER0006 servos, 850 nm IR LEDs, OV9281 USB camera (1.65 mm lens), Mean Well 12 V.
No endstop is fitted: homing is done by stalling at both mechanical stops.
Details and wiring: `docs/hardware/hardware.md`.

## Tests
`tools/run_tests.sh`, or in MATLAB: `cd matlab/tests; run_all_tests`, and `make -C firmware/tests`.

Read `AGENTS.md` before contributing. Do not commit recordings of eyes or calibration files
(`matlab/data/local/`, `matlab/config/local/` are git-ignored).
