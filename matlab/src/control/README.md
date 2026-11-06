# matlab/src/control

| | File | Purpose |
|---|---|---|
| RIG | `build_axis_model.m` | FIT0186 + belt (state space, ZOH) or SER0006 servo model |
| RIG (loop_margins ANALYSIS) | `design_pid.m`, `loop_margins.m` | Pole-placement PID, Bode / margins with delay |
| RIG | `pid_step.m` | PID used on the rig (set-point weighting, anti-windup, saturation, measured dt, feed-forward) |
| RIG (coordinator_setpoints ANALYSIS) | `refs_from_eye.m`, `coordinator_setpoints.m` | Head position -> carriage x + 2 LED pan angles (limits, slew) |
| RIG | `led_world_pos.m`, `led_aim.m` | LED positions (frame / carriage) and the servo angle that points an LED at the eye |
| ANALYSIS | `simulate_rig.m` | Closed-loop simulation of all 3 axes |
| RIG | `sysid_step.m` | Inertia identification from a voltage step |
| ANALYSIS | `step_metrics.m` | Rise / overshoot / settling |
| RIG | `rig_home_stall.m` | Stall homing: drive slowly into a mechanical stop until the encoder stops |
| ANALYSIS | `sim_carriage_step.m` | Carriage step on the nonlinear model with given loop periods (paper, controller tuning) |
| RIG | `rig_tick_wait.m`, `rig_move_to.m` | Real-time pacing and positioning for `run_rig` |
Equations and results: `docs/control-theory/control_design.md`. Validate changes with `run_simulation`
and `run_rig MODE='virtual'` before running on the hardware.

Status marks: see `../../README.md` (RIG = on the rig, TOOL = started by hand, LEDS = needs the LEDs, ANALYSIS = offline, SIM = virtual/tests).
