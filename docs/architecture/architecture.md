# Architecture

## Boundaries
```
 OV9281 USB camera ──USB──┐
                          │        PC / MATLAB  (all control, Ts = 20 ms target)
                          ├──> camera ─> eye_detection ─> head estimate ─> Kalman filter
                          │                    │                                 │
                          │                    └─> gaze_estimation (PCCR + calib) │
                          │                                                      v
                          │     control: refs_from_eye -> dead band/slew/limits -> PI
                          │                                                      │
 NUCLEO-H753ZI <──USB (ST-LINK VCP, 921600 8N1)── communication (protocol v1) <──┘
   │  firmware = I/O bridge + safety (watchdog; endstop gating unused, PB12 tied to GND)
   ├─ TIM4 PWM + DIR ─> MD10C ─> FIT0186 motor ─> GT2 belt ─> camera carriage
   ├─ TIM2 encoder  <─ FIT0186 encoder
   ├─ TIM3 PWM x2   ─> SER0006 servos (pan of LED 1, 2)
   ├─ GPIO x4       ─> NPN ─> IR LEDs 1-4
   └─ GPIO          <─ endstop input (not fitted; stall homing)
```
As built: `docs/hardware/hardware.md` ("As built"). On the rig now: face mode, servo 2 only,
no IR LEDs; `face_find` (Viola-Jones) replaces `eye_detection` in the diagram. Decision: `docs/decisions/0001-control-loop-in-matlab.md`. Protocol: `interface/protocol.md`.

## MATLAB modules (`matlab/src`)
| Module | Inputs -> outputs |
|---|---|
| camera | `camera_open/read/close`: webcam, video, images or synthetic -> mono uint8 frame + timestamp |
| eye_detection | `eye_detect`: frame (+ search windows) -> pupils, glints, confidence; `eye_tracker_*`: right/left identity, blink/lost logic; `head_estimate`: pixels -> head x, y, z, roll; face mode (rig now): `face_find` -> `face_mask` -> `face_head` (head x, distance); `eye_filter`: Kalman filter -> smoothed position, velocity, latency compensation |
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
- **Servo LEDs**: pan angle [rad] from straight ahead (+z), + toward the MAX side; pulse = zeroUs + dir * angle / (pi / 2000 us).

## Timing (control period `P.Ts` = 20 ms target)
The first design used a 10 ms period (100 Hz). On the rig the loop could not keep that, so
`P.Ts` is now 20 ms and the carriage bandwidth 1 Hz (`params.m`).

| Item | Notes |
|---|---|
| USB request/reply (11 + 17 bytes at 921600) | ST-LINK virtual COM port |
| PID + bookkeeping | small |
| Camera frame + face detection (every 2nd period, `P.cam.every`) | detection on a 1/3 downscaled frame (`P.face.scale`); a period with detection takes ~100 ms |
| Live view | rate-limited; set `SHOW = false` if loops overrun |

Measured on the rig (face mode, logs of 2026-11-05/06, `loopMs` in the log): median 46 ms,
mean 60 ms, 90th percentile 118 ms. The PID uses the measured dt; the log keeps the real time
(`L.tReal`). Speeding up the loop to 20 ms is still to do.
Camera latency: `P.cam.latency` = 0.10 s is a setting, not measured. In `run_rig` (serial) each
frame is compared with the carriage position at t - latency.

## Error handling
- Missing / corrupt reply: counted (`io.nErr`), motor command 0 for that period, link continues.
- Firmware watchdog (100 ms) acts even if MATLAB hangs.
- `run_rig` checks before and during motion: encoder pre-check before homing, travel check after
  homing at both stops, encoder guard at the limits, runaway check (`P.trk.maxErr`) and encoder-jump check -> STOP.
- Camera error or end of video: the script stops, motor 0, LEDs off (`onCleanup` -> STOP frame).
- Eyes / face lost: carriage holds its position until a new detection.
