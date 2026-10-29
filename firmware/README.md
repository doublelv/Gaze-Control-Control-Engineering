# firmware

Target: ST NUCLEO-H753ZI, C99. The firmware is an **I/O bridge** (decision 0001): it applies
MATLAB's commands (motor duty, 2 servo pulses, 4 LEDs), reports the encoder and the endstop,
and enforces the safety rules by itself. Protocol: `interface/protocol.md` (v1).

## Layout

| Path | Content |
|---|---|
| `App/app.c/.h` | Top level: UART ring buffer, frame handling, outputs, watchdog/endstop enforcement |
| `App/communication/` | Protocol v1 parser and encoders (portable) |
| `App/motor/`, `App/servo/` | Duty / pulse clamping and output |
| `App/safety/` | Watchdog and endstop gating |
| `App/config.h` | PWM counts, servo limits, watchdog time, endstop and direction settings **[CHECK]** |
| `App/bsp.h` | Hardware interface used by the modules |
| `App/bsp/bsp_stm32h7.c` | BSP with the STM32 HAL (compiled only in the CubeMX project) |
| `firmware.ioc`, `Core/`, `Drivers/`, `cmake/` | CubeMX-generated project (CMake). `main.c` calls `bsp_start()`, `app_init()`, `app_poll()` in USER CODE sections |
| `tests/` | Host unit tests (gcc), incl. the shared protocol vectors |

## Host tests (no board needed)

```
cd firmware/tests
make            # builds and runs 53 checks + syntax-checks the STM32 BSP against HAL stubs
```

## Creating the STM32 project (once the pins are decided)

1. STM32CubeIDE -> New STM32 Project -> Board Selector -> **NUCLEO-H753ZI**, "initialize all peripherals with default mode" = Yes.
2. Clock: HCLK 480 MHz; APB timer clocks 240 MHz **[CHECK]**. If different, change `MOTOR_PWM_COUNTS` (= timer clock / 20 kHz) and the TIM3 prescaler.
3. Peripherals (pick physical pins that offer the function; CubeMX shows them when you click a pin):

| Schematic J10 signal | CubeMX | Settings | User label |
|---|---|---|---|
| Motor 1 PWM -> MDD10A PWM1 | **TIM4 CH3** | PWM Generation CH3, Prescaler 0, Counter Period 11999 | - |
| Motor 1 DIR -> MDD10A DIR1 | GPIO_Output | | `MOTOR_DIR` |
| Encoder A (J10 pin 15, labelled "Encoder B" on the schematic) | **TIM2 CH1** | Combined Channels: Encoder Mode TI1 and TI2, Counter Period 0xFFFFFFFF, Input filter 6 | - |
| Encoder B | **TIM2 CH2** | (same timer) | - |
| Servo 1 / Servo 2 signal | **TIM3 CH1 / CH2** | PWM Generation, Prescaler 239 (1 MHz), Counter Period 19999 (50 Hz) | - |
| IR LED 1..4 (470R -> NPN base) | GPIO_Output | push-pull, low at reset | `LED_1` .. `LED_4` |
| Endstop at the MIN end | GPIO_Input | no pull (module has its own resistor) | `END_MIN` |
| ST-LINK virtual COM port | **USART3** | Asynchronous, 921600 baud, 8N1, NVIC USART3 global interrupt **enabled** | - |

### Pin assignment in `firmware.ioc` (all signals on Zio CN7, power from CN8: no soldering)

CN7 (20-pin, odd pins = left column, even = right column, pin 1 at the top; checked against the
ST NUCLEO-H753ZI (MB1364E) Zio pinout picture: D8 is PF3 on this board, not PF12):

| CN7 pin | Board label | MCU pin | Signal |
|---|---|---|---|
| 1 | D16 | PC6 | Servo 1 (TIM3_CH1) |
| 2 | D15 | PB8 | Motor PWM -> MDD10A PWM1 (TIM4_CH3) |
| 4 | D14 | PB9 | Motor DIR -> MDD10A DIR1 (`MOTOR_DIR`) |
| 7 | D19 | PB12 | Endstop MIN signal (`END_MIN`) |
| 8 | GND | GND | Common ground (MDD10A GND, LED/servo 5 V supply GND) |
| 10 | D13 | PA5 | Encoder A (TIM2_CH1) |
| 11 | D21 | PC7 | Servo 2 (TIM3_CH2) |
| 15 | D23 | PB3 | Encoder B (TIM2_CH2) |
| 16 | D10 | PD14 | IR LED 3 (`LED_3`) |
| 17 | D24 | PA4 | IR LED 4 (`LED_4`) |
| 18 | D9 | PD15 | IR LED 2 (`LED_2`) |
| 20 | D8 | PF3 | IR LED 1 (`LED_1`) |

Unused on CN7: 3 (PB15), 5 (PB13, Ethernet), 6 (VREF), 9 (PA15), 12 (PA6), 13 and 14 (both PB5), 19 (PB4).

CN8 (power only): pin 7 = 3V3 (encoder VCC, endstop +), pin 9 = 5V, pins 11/13 = GND.
Servos and IR LEDs are powered from the buck 5 V, not from the Nucleo; only their GND is shared.
USART3 (PD8/PD9) goes to the ST-LINK USB, no wire needed.

Clock: SYSCLK 480 MHz (PLL from HSI, voltage scale 0), APB1/APB2 timer clocks 240 MHz.

### Build and flash
- VS Code + STM32 extension: open the `firmware/` folder, configure preset `Debug`, build, flash.
- Command line: `cmake --preset Debug && cmake --build --preset Debug` (needs arm-none-eabi-gcc, CMake, Ninja),
  then flash `build/Debug/firmware.elf` with STM32CubeProgrammer.

4. Generate code. Add `App/` to the project (right-click project -> Properties -> C/C++ General -> Paths and Symbols:
   add `../App` and subfolders to Includes and **Source Location**), or copy `App/` into the project root.
5. Paste the three lines from `Core/Src/README.md` into the USER CODE sections of `main.c`.
6. Build, flash (Run -> Debug or the green Run button). The board's COM port appears as
   "STMicroelectronics STLink Virtual COM Port".

## First power-up checklist (actuators disconnected first)

1. Motor wires disconnected. In MATLAB: `matlab/scripts`, `run_rig` with `MODE='serial'`, `TASK='io_check'`.
   It must print `Link OK ... protocol v1, firmware 1.0` and 0 link errors.
2. Turn the motor shaft one turn by hand: count changes by ~700 (x1) or ~2800 (x4). If x4, set `P.mot.cpr` accordingly.
3. Press the endstop by hand: `endMin` must change. If inverted, flip `ENDSTOP_ACTIVE_HIGH` in `config.h`.
4. Motor connected, **belt off**: `io_check` pulses +2 V for 0.3 s; the carriage side of the pulley must turn toward MAX.
   If not, flip `MOTOR_DIR_POSITIVE_HIGH`.
5. Belt on: `TASK='sysid'`, copy the printed `mot.Jm` into `params_local.m`, then `TASK='track_synthetic'`.

## Safety behaviour (firmware, independent of MATLAB)
- Start-up: motor 0, LEDs off, servos 1500 us.
- No valid CMD for 100 ms -> motor 0, LEDs off (servos hold), reported once with the WDOG flag.
- Duty toward a pressed endstop is forced to 0, re-checked on every main-loop pass.
