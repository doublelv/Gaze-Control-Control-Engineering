# 0002 - Homing with one endstop at the MIN end

- Date: 2026-10-29
- Status: **superseded** on the rig by stall homing (2026-11-06, see the end)

## Context
The encoder is incremental, so at start-up the carriage position is unknown. The team has one endstop.

## Decision
Mount the endstop at the MIN end. At start-up MATLAB drives the carriage at 20 % duty
toward MIN until the switch is pressed; that position is zero. The MAX end is protected by
software limits (`P.belt.xMax`).

## Alternative (no switch)
`P.home.method = 'stall'`: drive slowly into a solid mechanical stop and detect that the
encoder stops changing. Needs a robust stop at the MIN end.

## Update 2026-11-06 (as built)
The endstop is not fitted; `PB12` (END_MIN) is tied to GND. The rig uses
`P.home.method = 'stall'` with `P.home.bothEnds = true`: drive at 20 % duty into the MIN stop,
then into the MAX stop; the measured travel must be within 5 cm of `P.belt.travel` (650 mm), and
the limits are centred on it. Before homing, a pre-check pushes toward MAX and requires the encoder
to count (a loose encoder wire made the carriage run away once).
