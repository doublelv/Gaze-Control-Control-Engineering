# matlab/src/visualization

| File | Purpose |
|---|---|
| `viz_live_init.m`, `viz_live_update.m` | Live window: camera + overlays, eye close-ups, gaze on screen, head/carriage history, status (rate-limited) |
| `viz_detection.m` | Overlays: search regions, candidates, pupils (green = right, cyan = left), glints |
| `plot_rig_log.m` | Plots of a `run_rig` log (reference/carriage/head, voltage, loop time, tracker state) |
| `viz_drawnow.m`, `viz_alive.m`, `median_valid.m`, `pick.m` | MATLAB/Octave-compatible helpers |
Units: image pixels for overlays, mm in the history plot. Rendering is kept separate from algorithms.
