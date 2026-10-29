# 0005 - Operating range with the OV9281 1.65 mm lens

- Date: 2026-10-29
- Status: **accepted** (camera fixed by the team)

## Context
f = 1.65 mm / 3 um = 550 px. A 4 mm pupil is 3.7 px across at 0.6 m; all LED glints merge into
about 1 bright spot.

## Decision
- Supported eye distance 0.35-0.65 m (`P.det.zRange` = [0.30 0.70]); beyond ~0.7 m the pupil is < 3 px
  and cannot be separated reliably from eyebrow fragments (robustness sweep: 12/12 pairs found up to
  0.6 m with noise 6-12, 0-5/12 at 0.75 m).
- The primary function is eye/head **position** tracking (sub-pixel pupil centre, mean error
  0.27 px in simulation). Gaze **direction** is coarse (decision 0004).
- Detector designed for small blobs: 2x min-downsampling for the full-frame search, enclosure
  test, symmetric glint masking for the sub-pixel centre.

## Consequences
Users sit 40-60 cm from the camera. A longer M7 lens would improve gaze accuracy roughly in
proportion to the focal length (the code only needs `P.cam.f_px`).
