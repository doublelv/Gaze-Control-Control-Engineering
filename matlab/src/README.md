# matlab/src

| Module | Main functions |
|---|---|
| `camera/` | `camera_open/read/close/list`, `synth_render`, `synth_scene`, `cam_project`, `screen_points`, `eye_trajectory`, `camera_measure` (simulation model) |
| `eye_detection/` | `eye_detect`, `eye_tracker_init/prior/update`, `head_estimate`, `eye_filter`, `cc_label`, `box_mean` |
| `gaze_estimation/` | `gaze_features`, `gaze_calib_collect/fit/apply/file/window`, `gaze_poly_terms`, `eye_model`, `glint_point`, `gaze_sim` |
| `control/` | `build_axis_model`, `design_pid`, `loop_margins`, `pid_step`, `refs_from_eye`, `coordinator_setpoints`, `simulate_rig`, `sysid_step`, `step_metrics`, `rig_tick_wait`, `rig_move_to` |
| `communication/` | `proto_*`, `encode_*`, `decode_*`, `rig_open/exchange/stop/close`, `transact`, `virtual_mcu_*` |
| `visualization/` | `viz_live_init/update`, `viz_detection`, `plot_rig_log`, helpers |
Every function has a help header (inputs, outputs, units). Units: SI (m, s, rad, V) unless noted;
image coordinates in pixels with v down (see `docs/architecture/architecture.md`).
