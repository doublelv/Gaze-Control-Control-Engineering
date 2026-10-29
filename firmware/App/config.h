/*
 * config.h - compile-time settings of the eye-tracking rig firmware.
 * Values marked [CHECK] must match the CubeMX configuration / hardware.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define FW_VERSION_MAJOR      1u
#define FW_VERSION_MINOR      0u

/* Motor PWM: TIM4 CH3 (PB8), 240 MHz timer clock / 20 kHz -> ARR = 11999       */
#define MOTOR_PWM_COUNTS      12000
/* Servo PWM: TIM3 CH1/CH2, 1 MHz tick, 50 Hz -> compare value = pulse in us */
#define SERVO_MIN_US          500u
#define SERVO_MAX_US          2500u
#define SERVO_DEFAULT_US      1500u
#define NUM_LEDS              4u

/* Safety: motor 0 and LEDs off when no valid CMD for this long */
#define WATCHDOG_MS           100u

/* Endstops wired (team has one, at the MIN end, see docs/decisions/0002) */
#define USE_END_MIN           1
#define USE_END_MAX           0
/* Level of the endstop signal when the lever is pressed: 1 = high      [CHECK with a multimeter] */
#define ENDSTOP_ACTIVE_HIGH   1

/* Motor direction: 1 = DIR pin high for positive duty                   [CHECK: + must move toward MAX] */
#define MOTOR_DIR_POSITIVE_HIGH 1

/* UART receive ring buffer */
#define RX_RING_SIZE          128u

#endif
