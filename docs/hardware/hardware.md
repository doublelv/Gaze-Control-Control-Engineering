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

## Wiring (team schematic, J10 = Nucleo connector numbering, not the real pin names)
| J10 | Signal | Nucleo peripheral | Notes |
|---|---|---|---|
| 5, 6 | Servo 1, 2 signal | TIM3 CH1, CH2 | servos powered from the **buck 5 V**, common GND |
| 13 | Motor PWM -> MDD10A PWM1 | TIM1 CH1 | |
| 14 | Motor DIR -> MDD10A DIR1 | GPIO `MOTOR_DIR` | |
| 15, 16 | Encoder A, B ("Encoder B" on pin 15 is a label typo) | TIM2 CH1, CH2 | encoder VCC 3.3 V |
| 17-20 | IR LED 1-4 | GPIO `LED_1..4` | 470 Ω -> NPN base |
| (add) | Endstop signal | GPIO `END_MIN` | 3.3 V / GND |
| USB | ST-LINK | USART3 921600 8N1 | to the laptop |
**Pins: TBD.** Choose physical pins with the listed timer functions in CubeMX (`firmware/README.md`).

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
