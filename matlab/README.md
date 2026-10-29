# matlab

Eye + head tracking, gaze estimation and real-time control of the rig. Written for
**MATLAB R2019b+**; all tests pass in **GNU Octave 8.4** (also used by CI).
Not yet run in MATLAB or on the hardware.

## Setup
1. MATLAB R2019b or newer. No toolboxes needed for simulation, tests and the virtual rig.
2. Real camera: install **MATLAB Support Package for USB Webcams** (Home -> Add-Ons -> Get Hardware Support Packages).
3. Real rig: flash the firmware (`../firmware/README.md`), then copy
   `config/params_local_example.m` to `config/local/params_local.m` and set your COM port and camera name.
4. In MATLAB: Current Folder = `matlab/scripts`. Every script calls `setup_paths` itself.

## Scripts (`scripts/`)
| Script | What it does | Needs | Moves hardware |
|---|---|---|---|
| `run_eye_demo` | Live eye + head tracking (+ gaze if calibrated) with overlays | camera, or `SOURCE='synthetic'` | no |
| `run_calibration` | 9-point gaze calibration, saved to `config/local/` | camera, or synthetic | no |
| `record_session` | Record IR frames to `data/local/` for tuning | camera | no |
| `tune_detector` | Detection statistics + overlays on recorded frames, parameter hints | recording, or synthetic | no |
| `run_rig` | Real-time control: `io_check`, `sysid`, `track_synthetic`, `track_camera` | rig, or `MODE='virtual'` | **yes in `serial` mode** |
| `run_simulation` | Offline control design study: PID, margins, step/tracking tests, gaze theory | nothing | no |

## Recommended order on the real hardware
1. `run_eye_demo` with `SOURCE='webcam'` (camera only, rig unpowered): are both pupils and glints marked?
2. `record_session` + `tune_detector` if detection is poor; adjust `P.det` in `params_local.m`.
3. `run_rig` `MODE='serial'`, `TASK='io_check'` (belt off): link, encoder, endstop, motor direction.
4. `TASK='sysid'` (belt on): copy the printed `P.mot.Jm` into `params_local.m`.
5. `TASK='track_synthetic'`: carriage follows a simulated head.
6. `run_calibration` with the LEDs on, then `TASK='track_camera'`.

## Data flow
camera -> `eye_detect` (pupils, glints) -> `eye_tracker_update` (left/right, blink, lost) ->
`head_estimate` (x, y, z, roll) -> `eye_filter` (Kalman, latency) -> `refs_from_eye`
(carriage x, LED tilts) -> `pid_step` -> `rig_exchange` -> Nucleo.
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

## Performance (simulation, synthetic images)
- Pupil centre: 0.27 px mean, < 1 px max error; both eyes found in 100 % of open-eye frames at 0.4-0.6 m.
- Head: lateral < 1 cm, distance +-4 mm (1 sigma), roll 0.3 deg; no false eyes during blinks.
- Carriage: 2 Hz bandwidth, PM 43 deg, GM 11 dB; camera-to-eye offset 10.9 mm RMS (with filter + feed-forward).
- Gaze: ~5 deg median after calibration at 0.6 m (limited by the 1.65 mm lens).
