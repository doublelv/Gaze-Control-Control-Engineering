# matlab/config

- `params.m`: every parameter with units and source. `[PLACEHOLDER]` = not in a datasheet, measure it;
  `[CHECK]` = datasheet interpretation, verify; `[CALIBRATE]` = set on the rig.
- `params_local_example.m`: copy to `local/params_local.m` for machine settings (COM port,
  camera name, measured values). Applied at the end of `params.m`; tests ignore it.
- `local/`: git-ignored. Also holds `gaze_calibration.mat` (personal data).
