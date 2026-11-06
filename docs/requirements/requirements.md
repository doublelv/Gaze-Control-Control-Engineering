# Requirements

Status column: **sim** = verified in simulation / offline test (`matlab/tests`, `firmware/tests`),
**hw** = verified on the rig, **open** = not verified, **not met** = tested and the acceptance is not reached.
All "sim" results come from synthetic images and models. They must be repeated on hardware.
State of the rig (2026-11-06): face tracking only (IR LEDs not mounted, servo 1 not wired), see
`docs/hardware/hardware.md`.

## Functional

| ID | Requirement | Acceptance | Verified by | Status |
|---|---|---|---|---|
| F1 | Detect both pupils in the OV9281 IR image | both pupils found in >= 95 % of open-eye frames at 0.35-0.65 m | `test_eye_detect`, `tune_detector` | sim (needs LEDs) |
| F2 | Detect the corneal reflections (glints) of the IR LEDs | glint found for >= 90 % of detected pupils | `test_eye_detect` | sim |
| F3 | Track the eyes over time, through blinks | no false eye during a blink; re-lock < 0.5 s after the eyes open | `test_tracker` | sim |
| F4 | Estimate the head position (x, y, distance z) and roll | lateral error < 10 mm, distance error < 20 mm, roll error < 2 deg | `test_tracker` | sim |
| F5 | Move the camera carriage to keep the head centred | camera-to-head offset RMS < 20 mm during the demo motion | `test_run_rig`, `run_simulation` sec. 3 | sim; rig (face mode) runs, offset not yet measured against this criterion |
| F6 | Aim the two servo LEDs at the eyes | eye inside the LED beam (+-17.5 deg) 100 % of the time | `run_simulation` sec. 3 | sim |
| F7 | Estimate the gaze point on the screen after a 9-point calibration | median error < 7 deg at 0.6 m (lens-limited, see decision 0005) | `test_gaze` | sim |
| F8 | Find the carriage zero position at start-up (homing) | homing within 20 s | `test_run_rig`, `test_link_virtual`, `run_rig` | hw (stall homing at both stops; no endstop fitted) |
| F9 | Identify the motor/load inertia from a step | inertia within 5 % on the virtual rig | `run_rig` TASK=`sysid` | sim; run on the rig 2026-11-04 (tau_m = 33.3 ms) |
| F10 | Run without hardware (virtual rig, synthetic camera, recorded video) | all scripts run in `virtual` / `synthetic` mode | `test_run_rig`, CI | sim |

## Performance / control

| ID | Requirement | Acceptance | Status |
|---|---|---|---|
| P1 | Carriage loop stable with the MATLAB loop + USB delay | phase margin >= 35 deg, gain margin >= 6 dB | model (`loop_margins`, delay 1.5 T): used on the rig (zeta 0.8): 44 deg / 20 dB at T = 20 ms, 35.5 deg / 13 dB at T = 46 ms; proposed (zeta 1.5): 54 deg / 16 dB and 42 deg / 10 dB. The rig runs stably |
| P2 | Carriage 50 mm step | settling (2 %) < 0.5 s, overshoot < 10 % | model, proposed tuning at T = 46 ms: overshoot 1.2 %, settling 0.69 s (settling not met). Rig: `TASK='step'` to do |
| P3 | Control loop timing | 20 ms period (first: 10 ms), overruns < 1 % | not met (hw): median 46 ms, mean 60 ms |
| P4 | Vision processing | a frame every 2nd period processed together with the control loop | not met (hw): face detection periods ~100 ms |

## Safety

| ID | Requirement | Acceptance | Status |
|---|---|---|---|
| S1 | Motor stops if MATLAB stops sending | motor 0 and LEDs off <= 100 ms after the last valid command | sim (`firmware/tests`, `test_link_virtual`) |
| S2 | No motion into a pressed endstop | duty toward the pressed endstop forced to 0, also between commands | sim (`firmware/tests`); not used on the rig (no endstop) |
| S3 | Software travel limits | carriage reference clipped to [xMin, xMax] | sim (`run_rig`) |
| S4 | Safe start-up | motor 0, LEDs off, servos 1500 us after reset | sim (`firmware/tests`) |
| S5 | Corrupt / foreign serial data ignored | frames with bad header, length or checksum dropped and counted | sim (both test suites) |
| S6 | Mains side of the PSU covered | LRS-150-12 terminals covered, switch + fuse | open (build): no switch yet |

## Operating conditions (derived from the hardware, see decision 0005)
- Eye distance 0.35-0.65 m (pupil >= 1.5 px radius with the 1.65 mm lens).
- IR illumination by the rig LEDs; no strong sunlight on the face.
- Same LEDs switched on during gaze calibration and use.
