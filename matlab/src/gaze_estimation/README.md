# matlab/src/gaze_estimation

| | File | Purpose |
|---|---|---|
| LEDS | `gaze_features.m` | PCCR vector per eye, scaled to metres at the eye (x z/f) |
| LEDS | `gaze_calib_collect.m` | Shows targets, records median PCCR per target (real or synthetic camera, headless option) |
| LEDS | `gaze_calib_fit.m` | 2nd-order polynomial per eye, fit residual and leave-one-out error [deg] |
| LEDS | `gaze_calib_apply.m` | PCCR -> screen point (m, normalised, pixels), binocular average |
| LEDS | `gaze_calib_window.m` | Full-screen black window with the animated dot |
| RIG (loads only) | `gaze_calib_file.m` | Save/load in `config/local/` (personal data, git-ignored) |
| SIM | `eye_model.m`, `glint_point.m` | 3-D eye and corneal reflection geometry (renderer, `gaze_sim`) |
| ANALYSIS | `gaze_sim.m` | Monte-Carlo study of 3-D (Shih & Liu style) gaze reconstruction vs pixel noise; not line-by-line checked against the paper |
Gaze representation: point on the screen plane (world x, y in m) and normalised screen coordinates.
Calibration validity: same user, same LEDs, similar distance; recalibrate otherwise.

Status marks: see `../../README.md` (RIG = on the rig, TOOL = started by hand, LEDS = needs the LEDs, ANALYSIS = offline, SIM = virtual/tests).
