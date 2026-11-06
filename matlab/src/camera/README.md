# matlab/src/camera

| | File | Purpose |
|---|---|---|
| RIG | `camera_open.m` / `camera_read.m` / `camera_close.m` | Frame sources: `webcam` (OV9281), `video`, `images`, `synthetic`. Mono uint8 + timestamp; clear errors (missing support package, no camera, wrong name, end of file) |
| TOOL | `camera_list.m` | Lists connected cameras and resolutions |
| SIM | `synth_render.m` | Synthetic IR face: pupils, iris, sclera, glints from the real LED positions (spherical cornea), eyebrows/nostrils as distractors, blinks, noise, anti-aliased |
| SIM | `synth_scene.m` | Ground-truth scenes: `demo` (moving head, roll, gaze hops, blinks), `calib` (fixed target) |
| SIM | `cam_project.m` | Pinhole projection (camera on the carriage) |
| SIM | `screen_points.m` | Calibration target grid on the screen |
| ANALYSIS | `eye_trajectory.m`, `camera_measure.m` | Head path and camera model for `run_simulation` |
Synthetic frames are the only test fixtures (no recordings of real eyes in the repository).

Status marks: see `../../README.md` (RIG = on the rig, TOOL = started by hand, LEDS = needs the LEDs, ANALYSIS = offline, SIM = virtual/tests).
