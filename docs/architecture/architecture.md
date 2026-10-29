# Architecture

## Boundaries
```
 OV9281 USB camera ──USB──┐
                          │        PC / MATLAB  (all control, 100 Hz)
                          ├──> camera ─> eye_detection ─> head estimate ─> Kalman filter
                          │                    │                                 │
                          │                    └─> gaze_estimation (PCCR + calib) │
                          │                                                      v
                          │     control: refs_from_eye -> slew/limits -> PID + velocity FF
                          │                                                      │
 NUCLEO-H753ZI <──USB (ST-LINK VCP, 921600 8N1)── communication (protocol v1) <──┘
   │  firmware = I/O bridge + safety (watchdog, endstop gating)
   ├─ TIM4 PWM + DIR ─> MDD10A ─> FIT0186 motor ─> GT2 belt ─> camera carriage
   ├─ TIM2 encoder  <─ FIT0186 encoder
   ├─ TIM3 PWM x2   ─> SER0006 servos (tilt of LED 1, 2)
   ├─ GPIO x4       ─> NPN ─> IR LEDs 1-4
   └─ GPIO          <─ endstop (MIN end)
```
Decision: `docs/decisions/0001-control-loop-in-matlab.md`. Protocol: `interface/protocol.md`.

## MATLAB modules (`matlab/src`)
| Module | Inputs -> outputs |
|---|---|
| camera | `camera_open/read/close`: webcam, video, images or synthetic -> mono uint8 frame + timestamp |
| eye_detection | `eye_detect`: frame (+ search windows) -> pupils, glints, confidence; `eye_tracker_*`: right/left identity, blink/lost logic; `head_estimate`: pixels -> head x, y, z, roll; `eye_filter`: Kalman filter -> smoothed position, velocity, latency compensation |
| gaze_estimation | `gaze_features` (PCCR), `gaze_calib_*` (collect, fit, apply, save/load); `gaze_sim` (3-D model study) |
| control | plant models, PID design, margins, `pid_step`, `refs_from_eye`, simulation, system identification, pacing |
| communication | protocol v1 codec, serial transport, virtual MCU |
| visualization | live view, detection overlays, log plots |

## Coordinate frames and units
- **World**: metres; x right, y up, z from the rig toward the user. Camera centre at
  `P.geo.cam + [xCar; 0; 0]`; screen in the plane z = 0, centre `P.screen.centre`.
- **Carriage**: x = r * theta (r = 12.7 mm GT2 40T pulley); theta = output-shaft angle [rad] from the encoder.
- **Image**: pixels; u right, v **down**; pixel (row r, col c) has its centre at (c - 0.5, r - 0.5).
  Principal point = image centre, f = 550 px (1.65 mm / 3 um) **[CHECK by camera calibration]**.
- **Eyes**: index 1 = right in the image (the user's left eye), 2 = left in the image.
- **Servo LEDs**: tilt angle [rad] above horizontal; pulse = zeroUs + dir * angle / (pi / 2000 us).

## Timing budget (per 10 ms control period)
| Item | Budget | Notes |
|---|---|---|
| USB request/reply (11 + 17 bytes at 921600) | ~1-2 ms | ST-LINK VCP latency dominates **[measure: loop_ms in run_rig]** |
| PID + bookkeeping | < 0.5 ms | |
| Camera frame + detection (every 3rd period) | < 25 ms | spread: the period overruns when a frame is processed; the PID uses the measured dt |
| Live view | rate-limited to 10 Hz | set `SHOW = false` if loops overrun |
Camera latency (exposure + MJPG + USB) is compensated in `eye_filter` with `P.cam.latency` **[measure]**.

## Error handling
- Missing / corrupt reply: counted (`io.nErr`), motor command 0 for that period, link continues.
- Firmware watchdog (100 ms) and endstop gating act even if MATLAB hangs.
- Camera error or end of video: the script stops, motor 0, LEDs off (`onCleanup` -> STOP frame).
- Eyes lost: carriage holds its position until a new eye pair is found.
