/* motor.c - see motor.h */
#include "motor.h"
#include "../config.h"
#include "../bsp.h"

int32_t motor_clamp(int32_t duty)
{
    if (duty >  MOTOR_PWM_COUNTS) return  MOTOR_PWM_COUNTS;
    if (duty < -MOTOR_PWM_COUNTS) return -MOTOR_PWM_COUNTS;
    return duty;
}

void motor_apply(int32_t duty)
{
    duty = motor_clamp(duty);
    bool positive = duty >= 0;
    uint32_t mag = (uint32_t)(positive ? duty : -duty);
    bsp_motor_write(mag, MOTOR_DIR_POSITIVE_HIGH ? positive : !positive);
}
