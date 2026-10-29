/* servo.h - SER0006 micro servos, 50 Hz, pulse width in us. */
#ifndef APP_SERVO_H
#define APP_SERVO_H
#include <stdint.h>
uint16_t servo_clamp(uint16_t us);              /* to [SERVO_MIN_US, SERVO_MAX_US] */
void     servo_apply(uint8_t channel, uint16_t us);
#endif
