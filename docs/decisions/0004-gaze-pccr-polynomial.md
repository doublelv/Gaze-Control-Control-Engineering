# 0004 - Gaze estimation: PCCR vector + per-user polynomial calibration

- Date: 2026-10-29
- Status: **accepted**

## Context
The project started from Shih & Liu (2004, stereo 3-D model). The rig has one camera, and
its 1.65 mm lens images the eye with very few pixels (decision 0005).

## Alternatives
1. 3-D model (cornea centre from glints, pupil, optical axis): needs per-user cornea radius,
   and `gaze_sim.m` shows a 3-13 deg error already at 0.1-0.5 px noise with f = 550 px.
2. Stereo 3-D model (Shih & Liu): needs a second camera.
3. **PCCR (pupil centre - corneal reflection) vector + 9-point polynomial calibration** (chosen).

## Decision
Option 3 (`matlab/src/gaze_estimation`). The PCCR vector is scaled by z/f (distance from the
interpupillary distance) to reduce its dependence on the viewing distance; 2nd-order polynomial
per eye; binocular average; leave-one-out error reported after each calibration.

## Consequences
- Simulated accuracy at 0.6 m: ~5 deg median on new targets (resolution-limited).
- Calibration is personal data: stored in `matlab/config/local/` (git-ignored).
- Keep the same LEDs on as during calibration (the glint centroid moves otherwise).
