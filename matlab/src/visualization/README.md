# matlab/src/visualization

| | File | Purpose |
|---|---|---|
| RIG | `viz_live_init.m`, `viz_live_update.m` | Live window: camera + overlays, eye close-ups, gaze on screen, head/carriage history, status (rate-limited) |
| RIG | `viz_detection.m` | Overlays: search regions, candidates, pupils (green = right, cyan = left), glints |
| RIG | `plot_rig_log.m` | Plots of a `run_rig` log (reference/carriage/head, voltage, loop time, tracker state) |
| RIG | `viz_drawnow.m`, `viz_alive.m`, `median_valid.m`, `pick.m` | MATLAB/Octave-compatible helpers |
Units: image pixels for overlays, mm in the history plot. Rendering is kept separate from algorithms.

Status marks: see `../../README.md` (RIG = on the rig, TOOL = started by hand, LEDS = needs the LEDs, ANALYSIS = offline, SIM = virtual/tests).
