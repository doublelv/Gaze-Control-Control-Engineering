# MATLAB / STM32 interface contract - protocol v1

**Status: accepted** (decision `docs/decisions/0003-protocol-v1.md`).
Implementations: `matlab/src/communication/` and `firmware/App/communication/`.
Both are tested against `test-vectors/protocol_v1.txt`.

## Roles
- **MATLAB (PC)** owns all control: camera, eye/head tracking, carriage PID, servo angles, LEDs.
- **STM32 (NUCLEO-H753ZI)** is an I/O bridge: applies the last command, reads encoder and
  endstop, enforces the safety rules below.
- MATLAB sends one request every 10 ms (100 Hz) and waits for exactly one reply.

## Transport and lifecycle
- ST-LINK virtual COM port, **921600 baud, 8N1**, no flow control, read timeout 50 ms (MATLAB).
- Start-up: MATLAB opens the port, flushes, sends HELLO (up to 3 tries) and requires
  `proto == 1`. Otherwise it stops with an error naming the port and baud rate.
- Shut-down: MATLAB sends STOP (motor 0, LEDs off) and closes the port. If MATLAB disappears, the
  firmware watchdog does the same after 100 ms.
- Reconnection: open the port again (new HELLO); the firmware needs no reset.

## Frame (all multi-byte fields little-endian)
```
AA 55 | type (u8) | len (u8, 0..16) | payload[len] | checksum (u8)
checksum = (type + len + sum(payload)) mod 256
```

## Messages
| Type | Dir | Payload | Reply |
|---|---|---|---|
| `0x01` CMD | PC->MCU | `int16 duty` (PWM counts, -12000..12000 = -12..+12 V, sign = direction), `u16 servoA_us`, `u16 servoB_us` (clamped 500..2500), `u8 ledMask` (bit i = LED i+1), `u8 seq` | STATE |
| `0x02` HELLO | PC->MCU | `u8 seq` | HELLO_REPLY |
| `0x03` STOP | PC->MCU | `u8 seq` -> motor 0, LEDs off, servos hold | STATE |
| `0x81` STATE | MCU->PC | `int32 count` (encoder, output 700 CPR x1/x4 [CHECK]), `u8 flags`, `u8 seq` (echo), `u32 t_us`, `u16 rxErrors` | - |
| `0x82` HELLO_REPLY | MCU->PC | `u8 proto` (=1), `u8 fwMajor`, `u8 fwMinor`, `u8 seq`, `u32 t_us` | - |

STATE flags: bit0 MIN endstop pressed, bit1 MAX endstop pressed, bit2 watchdog tripped since the
previous STATE, bit3 duty forced to 0 by an endstop, bit4 malformed frame since the previous STATE.

## Validation and errors
- Receivers resynchronise on `AA 55`; frames with len > 16 or a wrong checksum are dropped and
  counted (`rxErrors`, flag bit4). Valid frames of unknown type/length are counted too, without a reply.
- MATLAB drops replies with a wrong type or `seq` and counts them as link errors. For that period it
  sends duty 0. There are no retries: the next period sends a fresh command.

## Safety (firmware, independent of MATLAB)
- Start-up: motor 0, LEDs off, servos 1500 us.
- **Watchdog:** no valid CMD for 100 ms -> motor 0, LEDs off (servos hold), reported once via bit2.
- Duty toward a pressed endstop is forced to 0, re-checked on every main-loop pass (bit3).

## Units and frames
Duty in PWM counts of `MOTOR_PWM_COUNTS` (= P.drv.pwmCounts), servo pulses in microseconds,
encoder counts of the FIT0186 output shaft (carriage x = r * 2*pi * count / CPR).
