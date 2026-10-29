# matlab/src/eye_detection

| File | Purpose |
|---|---|
| `eye_detect.m` | Dark-pupil + glint detector. Full frame (2x min-downsampled) or ROI search; local-contrast mask; component shape/size/darkness/enclosure tests -> confidence; eye-pair selection by interpupillary distance and roll; sub-pixel centre with symmetric glint masking; glint centroids; PCCR vector |
| `eye_tracker_init/prior/update.m` | Right/left identity, prediction, search windows, physical jump gate, states search / track / blink / lost |
| `head_estimate.m` | Pixels -> head x, y, z (from pupil spacing) and roll in the world frame |
| `eye_filter.m` | Kalman filter (position + velocity), latency prediction, saccade-jump gating |
| `cc_label.m`, `box_mean.m` | Toolbox-free connected components and box filter |
Assumptions: dark pupil under 850 nm IR, pupils 1.5-7 px radius (0.35-0.65 m). Invalid detections
(blink, occlusion) give `obs.seen = false`; the tracker coasts, then searches the full frame.
Tuning: `P.det`, `P.trk` in `params.m`, with `scripts/tune_detector.m`.
