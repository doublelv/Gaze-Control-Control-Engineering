# matlab/tests

`run_all_tests` (MATLAB or Octave, from this folder) runs every `test_*.m`:

| Test | Checks |
|---|---|
| `test_protocol` | codec vs `interface/test-vectors/protocol_v1.txt` (valid + invalid), clipping |
| `test_link_virtual` | handshake, round trips, endstop gating, watchdog, corrupt frames (virtual MCU) |
| `test_cc_label` | connected components |
| `test_eye_detect` | pupils/glints on synthetic frames (< 1 px), clean-disc sub-pixel accuracy, blinks |
| `test_tracker` | camera -> tracker -> head on the moving synthetic scene (lock, blinks, x/z/roll errors) |
| `test_gaze` | headless synthetic calibration, leave-one-out and new-target errors, save/load |
| `test_pid_design` | phase / gain margins |
| `test_run_rig` | the `run_rig` script in virtual mode: io_check, track_synthetic, track_camera |

Fixtures are synthetic (rendered faces, generated vectors); no personal recordings.
Tests ignore `config/local/params_local.m`. Hardware results are reported separately.
