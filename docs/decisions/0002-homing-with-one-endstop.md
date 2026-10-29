# 0002 - Homing with one endstop at the MIN end

- Date: 2026-10-29
- Status: **accepted** (one endstop available, 2026-10-29)

## Context
The encoder is incremental, so at start-up the carriage position is unknown. The team has one endstop.

## Decision
Mount the endstop at the MIN end. At start-up MATLAB drives the carriage at 20 % duty
toward MIN until the switch is pressed; that position is zero. The MAX end is protected by
software limits (`P.belt.xMax`).

## Alternative (no switch)
`P.home.method = 'stall'`: drive slowly into a solid mechanical stop and detect that the
encoder stops changing. Needs a robust stop at the MIN end.
