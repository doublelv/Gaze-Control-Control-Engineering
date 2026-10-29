/*
 * bsp.h - hardware abstraction used by the application modules.
 * Target implementation: bsp/bsp_stm32h7.c (STM32 HAL).
 * Host tests:            firmware/tests/fake_bsp.c
 */
#ifndef APP_BSP_H
#define APP_BSP_H
#include <stdint.h>
#include <stdbool.h>

void     bsp_motor_write(uint32_t compare, bool positive);  /* PWM compare 0..MOTOR_PWM_COUNTS, direction */
void     bsp_servo_write(uint8_t channel, uint16_t pulse_us); /* channel 0 = A, 1 = B */
void     bsp_led_write(uint8_t mask);                         /* bit i = LED i+1 */
int32_t  bsp_encoder_read(void);                              /* 32-bit quadrature counter */
uint8_t  bsp_endstops_read(void);                             /* raw levels: bit0 MIN, bit1 MAX (1 = high) */
uint32_t bsp_micros(void);                                    /* free-running microsecond clock */
uint32_t bsp_millis(void);
void     bsp_uart_send(const uint8_t *data, uint16_t len);

#endif
