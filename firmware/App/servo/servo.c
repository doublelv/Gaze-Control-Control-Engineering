/* servo.c - see servo.h */
#include "servo.h"
#include "../config.h"
#include "../bsp.h"

uint16_t servo_clamp(uint16_t us)
{
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

void servo_apply(uint8_t channel, uint16_t us)
{
    bsp_servo_write(channel, servo_clamp(us));
}
