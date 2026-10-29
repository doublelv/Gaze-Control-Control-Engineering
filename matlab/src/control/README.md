# matlab/src/control

| File | Purpose |
|---|---|
| `build_axis_model.m` | FIT0186 + belt (state space, ZOH) or SER0006 servo model |
| `design_pid.m`, `loop_margins.m` | Pole-placement PID, Bode / margins with delay |
| `pid_step.m` | PID used on the rig (set-point weighting, anti-windup, saturation, measured dt, feed-forward) |
| `refs_from_eye.m`, `coordinator_setpoints.m` | Head position -> carriage x + 2 LED tilt angles (limits, slew) |
| `simulate_rig.m` | Closed-loop simulation of all 3 axes |
| `sysid_step.m` | Inertia identification from a voltage step |
| `step_metrics.m` | Rise / overshoot / settling |
| `rig_tick_wait.m`, `rig_move_to.m` | Real-time pacing and positioning for `run_rig` |
Equations and results: `docs/control-theory/control_design.md`. Validate changes with `run_simulation`
and `run_rig MODE='virtual'` before running on the hardware.
