# 0001 - Control loop runs in MATLAB, Nucleo is an I/O bridge

- Date: 2026-10-29
- Status: **accepted** (owner of the MATLAB + firmware work, 2026-10-29)

## Context
The carriage (FIT0186 + GT2 belt) and two SER0006 servos must follow the eye seen by the
OV9281 camera, which is processed in MATLAB on the PC.

## Alternatives
1. PID on the Nucleo at 1 kHz, MATLAB only sends target positions.
2. **PID in MATLAB at 100 Hz, Nucleo only does I/O** (chosen).
3. Code generation from Simulink to the Nucleo.

## Decision
Option 2.

## Rationale
- Simulation (`matlab/scripts/run_simulation.m`): with 10 ms loop + USB delay a 2 Hz carriage
  bandwidth gives 43 deg phase margin / 11 dB gain margin; a 50 mm step settles in 0.38 s vs ~0.24 s for option 1.
  The rotor inertia dominates (98 %), so the slower loop costs little.
- One place for all control code; firmware stays small.

## Consequences
- Firmware must have a watchdog (stop the motor if MATLAB stops sending).
- Protocol: `interface/protocol.md` (v1).
- Can move to option 1 later without changing the MATLAB vision code.
