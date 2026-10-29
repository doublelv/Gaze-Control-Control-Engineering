/* motor.h - FIT0186 via MDD10A (sign-magnitude PWM + DIR). */
#ifndef APP_MOTOR_H
#define APP_MOTOR_H
#include <stdint.h>
int32_t motor_clamp(int32_t duty);         /* to [-MOTOR_PWM_COUNTS, MOTOR_PWM_COUNTS] */
void    motor_apply(int32_t duty);         /* clamps, writes compare + direction */
#endif
