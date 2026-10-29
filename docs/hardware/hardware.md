# Hardware

## Bill of materials (ordered)
| Part | Use | Key data |
|---|---|---|
| NUCLEO-H753ZI (STM32H753, Cortex-M7) | I/O bridge | ST-LINK virtual COM port |
| Mean Well LRS-150-12 | 12 V supply | 12.5 A. **Mains terminals must be covered** |
| Cytron MDD10A (and MD10C spare) | motor driver | 10 A/ch, 5-30 V, sign-magnitude PWM + DIR, 3.3 V logic |
| DFRobot FIT0186 | carriage motor | 12 V, 43.8:1, 251 rpm, 18 kg·cm stall, 7 A stall, encoder 16 CPR motor = 700 CPR output, 6 mm D-shaft |
| GT2 6 mm belt, 40T pulley (6.35 mm bore), smooth idler (5 mm) | linear drive | 80 mm per output revolution. **Pulley bore 6.35 on a 6 mm shaft: shim or sleeve** |
| V-slot wheels (24 mm), M5 T-nuts for 2020 | carriage guide | 2020 **V-slot** profile needed |
| 2 x DFRobot SER0006 | LED tilt | 4.8-6 V, 0.156 N·m at 6 V, 0.12 s/60 deg, 7 us dead band |
| 4 x Würth IR LED 850 nm + 68 Ω | illumination | 50 mA, Vf <= 2.2 V, 35 deg half-intensity angle **[CHECK full/half]** |
| 4 x NPN + 470 Ω base + 10 kΩ pull-down | LED switches | from the team schematic |
| Buck converter 12 V -> 5 V | servos (+ LEDs) | ordered by the team (TinyTronics) **[CHECK current >= 2 A]** |
| Creality endstop (1 piece) | homing at the MIN end | |
| Waveshare OV9281 USB camera (A) | eye camera | mono, no IR filter, global shutter, 1280x720 @ 120 fps (MJPG), 1.65 mm lens, 105 deg FOV |

## Wiring (team schematic J10 -> NUCLEO-H753ZI, jumper wires on Zio CN7/CN8, no soldering)
| J10 | Schematic signal | Goes to | Nucleo pin | Header |
|---|---|---|---|---|
| 1 | USB | laptop | ST-LINK USB (USART3 921600 8N1 inside) | USB cable |
| 2 | +5VDC | IR LED anodes via 68 Ω | 5V | CN8 pin 9 |
| 3 | +3V3 | encoder VCC (J5-4) | 3V3 | CN8 pin 7 |
| 4 | GND | common ground (MDD10A, buck, encoder) | GND | CN8 pin 11 or CN7 row 4 right |
| 5 | Servo 1 signal | J1-2 | PC6, TIM3_CH1 | CN7 row 1 left |
| 6 | Servo 2 signal | J3-2 | PC7, TIM3_CH2 | CN7 row 6 left |
| 7-12 | not connected | | | |
| 13 | Motor 1 PWM | MDD10A PWM1 (J2-4) | PB8, TIM4_CH3, 20 kHz | CN7 row 1 right (D15) |
| 14 | Motor 1 DIR | MDD10A DIR1 (J2-5) | PB9, `MOTOR_DIR` | CN7 row 2 right (D14) |
| 15 | "Encoder B" (wire goes to ENCODER A, J5-5) | encoder A | PA5, TIM2_CH1 | CN7 row 5 right (D13) |
| 16 | "Encoder 3 B" (wire goes to ENCODER B, J5-6) | encoder B | PB3, TIM2_CH2 | CN7 row 8 left |
| 17 | IR LED 1 | R8 -> Q4 -> LED **D4** | PF3, `LED_1` | CN7 row 10 right (D8) |
| 18 | IR LED 2 | R7 -> Q3 -> LED **D3** | PD15, `LED_2` | CN7 row 9 right (D9) |
| 19 | IR LED 3 | R6 -> Q2 -> LED **D2** | PD14, `LED_3` | CN7 row 8 right (D10) |
| 20 | IR LED 4 | R5 -> Q1 -> LED **D1** | PA4, `LED_4` | CN7 row 9 left |
| (add) | Endstop S / + / - | Creality endstop, MIN end | PB12 `END_MIN` / IOREF 3.3 V / GND | CN7 row 4 left / CN8 pin 3 / CN8 pin 13 |

Notes:
- "IR LED i" in software (ledMask bit i-1, `P.led` column i) = J10 "IR LED i", which drives schematic LED D(5-i).
  `P.led.servo` must name the LEDs that are physically on servo 1 and servo 2.
- Servo power (J1-1, J3-1) comes from the buck 5 V; motor power is the 12 V supply. Only GND is shared with the Nucleo.
- The endstop is not in the schematic yet: add it (3 wires).
- Unused CN7 pins: PB15, PB13 (Ethernet), VREFP, PA15, PA6, PB5 (twice), PB4.

## Power budget
| Load | Supply | Current |
|---|---|---|
| FIT0186 | 12 V via MDD10A | 0.35 A no-load, 7 A stall (homing limited to ~1.4 A) |
| 2 x SER0006 | buck 5 V | up to ~0.7 A each when stalled |
| 4 x IR LED | 5 V (preferably buck, not the Nucleo 5 V pin) | 4 x 51 mA = 0.2 A |
| Camera | laptop USB | < 0.3 A |
LED resistor check: from 5 V, I = (5 - 1.5)/68 = 51 mA, P = 180 mW (0.25 W resistor: OK, warm).
**Never connect LED + 68 Ω to 12 V** (154 mA, 1.6 W).

## Mechanics to verify
- Rail length and the usable travel -> `P.belt.xMin/xMax`.
- A solid stop at the MIN end (homing drives into the switch at 2.4 V).
- Carriage mass (camera + LEDs + servos) -> `P.belt.mCar` (minor effect).
- LED positions relative to the camera -> `P.led.pos`. Camera and LEDs must not be on one line (needed for the 3-D gaze model study only).
