# matlab

Eye + head tracking, gaze estimation and real-time control of the rig. Written for
**MATLAB R2019b+**; all tests pass in MATLAB and in **GNU Octave 8.4** (also used by CI).
Used on the rig with MATLAB on Windows (face tracking); see the status legend below.

## Status legend (used in every README of this folder)
| Mark | Meaning |
|---|---|
| RIG | runs on the real rig (`run_rig` with `MODE='serial'`) |
| TOOL | tool that is started by hand (not called by other code) |
| LEDS | written and tested offline, needed once the IR LEDs are mounted (eye tracking, gaze) |
| ANALYSIS | offline analysis / simulation / paper figures |
| SIM | only for `MODE='virtual'` and the tests (simulated Nucleo, synthetic camera) |
Found by running `run_rig` (all tasks) with the MATLAB/Octave profiler and by following the calls
from every script.

## Setup
1. MATLAB R2019b or newer. No toolboxes needed for simulation, tests and the virtual rig.
2. Real camera: install **MATLAB Support Package for USB Webcams** (Home -> Add-Ons -> Get Hardware Support Packages).
3. Real rig: flash the firmware (`../firmware/README.md`), then copy
   `config/params_local_example.m` to `config/local/params_local.m` and set your COM port and camera name.
4. In MATLAB: Current Folder = `matlab/scripts`. Every script calls `setup_paths` itself.

## Scripts (`scripts/`)
| | Script | What it does | Needs | Moves hardware |
|---|---|---|---|---|
| RIG | `run_rig` | Real-time control: `io_check`, `sysid`, `track_synthetic`, `step`, `track_camera` | rig, or `MODE='virtual'` | **yes in `serial` mode** |
| RIG | `setup_paths` | Adds `src/*`, `config`, `config/local` to the path (called by every script) | - | no |
| TOOL | `servo_test` | Finds `P.srv.zeroUs` and `P.srv.dir` of the pan servos interactively | rig | servos only |
| TOOL | `motor_test` | Open-loop motor test (forward, stop, backward, stop), prints encoder counts and speed; belt off or carriage in the middle, no homing | rig | **yes** |
| TOOL | `run_eye_demo` | Live eye + head tracking (+ gaze if calibrated) with overlays | camera, or `SOURCE='synthetic'` | no |
| TOOL | `record_session` | Record IR frames to `data/local/` for tuning | camera | no |
| TOOL | `tune_detector` | Detection statistics + overlays on recorded frames, parameter hints | recording, or synthetic | no |
| LEDS | `run_calibration` | 9-point gaze calibration, saved to `config/local/` | camera + LEDs, or synthetic | no |
| ANALYSIS | `run_simulation` | Offline control design study: PID, margins, step/tracking tests, gaze theory | nothing | no |
| ANALYSIS | `make_paper_figures` | Plant, margins, step and latency-compensation figures (model) | nothing | no |
| ANALYSIS | `compare_log_sim` | Replays a real `run_rig` log through the model (sim-to-real check) | a log | no |
| ANALYSIS | `plot_paper_logs` | Figures from real logs: sysid fit, real vs model, loop-time histogram, real step | logs | no |

## Order on the real hardware
Before every session: plug in the Nucleo USB first, then the 12 V supply (it has no switch: unplug
it to switch off). If MATLAB reports "No HELLO reply": press RESET on the Nucleo, then
`delete(serialportfind); clear all`. The yellow serialport timeout warning can be ignored.

| Step | Status |
|---|---|
| 1. `run_rig` `TASK='io_check'` (belt off): link, encoder, motor direction | done |
| 2. `TASK='sysid'`: rotor inertia (`P.mot.Jm` in `params.m`) | done (2026-11-04) |
| 3. `TASK='track_camera'` with `P.trk.source='face'` | works |
| 4. `TASK='step'`: step response for the sim-to-real comparison | to do |
| 5. `servo_test`, then the servo values in `params.m` | servo 2 partly, servo 1 not wired |
| 6. LEDs mounted: `run_eye_demo` (`SOURCE='webcam'`), `tune_detector`, `run_calibration`, `P.trk.source='eyes'` | to do |

## Data flow
Face mode (used now): camera -> `face_find` (Viola-Jones) -> `face_mask` -> `face_head`
(head x, distance) -> `eye_filter` -> `refs_from_eye` -> `pid_step` -> `rig_exchange` -> Nucleo.

Eye mode (with LEDs): camera -> `eye_detect` (pupils, glints) -> `eye_tracker_update` (left/right, blink, lost) ->
`head_estimate` (x, y, z, roll) -> `eye_filter` (Kalman, latency) -> `refs_from_eye`
(carriage x, LED pan angles) -> `pid_step` -> `rig_exchange` -> Nucleo.
Gaze: `gaze_features` (PCCR) -> `gaze_calib_apply` -> screen point.

## Folders
| Folder | Content |
|---|---|
| `config/` | `params.m` (all parameters, `[PLACEHOLDER]` = to measure, `[CHECK]` = to verify), `params_local_example.m`; `local/` is git-ignored (COM port, calibration) |
| `src/` | modules, see `src/README.md` |
| `scripts/` | entry points above |
| `tests/` | `run_all_tests` (8 suites, ~3 min in Octave, synthetic data only) |
| `data/local/` | recordings (git-ignored, personal data) |
Outputs (figures, logs) go to `../artifacts/` (git-ignored).

## Results
On the rig (face tracking, 2026-11-05/06):
- Loop period: median 46 ms, mean 60 ms (frames with face detection ~100 ms); target 20 ms.
- Carriage PID at 1 Hz (zeta 0.8) was used; open-loop identification: tau_m = 33.3 ms.
- Replaying an 82 s log through the model (`compare_log_sim`): 9.4 mm RMS difference.

Simulation only (synthetic images, from the first design study at 10 ms / 2 Hz; not repeated on the rig):
- Pupil centre: 0.27 px mean, < 1 px max error; both eyes found in 100 % of open-eye frames at 0.4-0.6 m.
- Head: lateral < 1 cm, distance +-4 mm (1 sigma), roll 0.3 deg; no false eyes during blinks.
- Gaze: ~5 deg median after calibration at 0.6 m (limited by the 1.65 mm lens).
