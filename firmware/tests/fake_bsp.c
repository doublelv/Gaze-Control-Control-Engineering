/* fake_bsp.c - host implementation of bsp.h for unit tests. */
#include "fake_bsp.h"
#include <string.h>

fake_bsp_t FB;

void fake_reset(void) { memset(&FB, 0, sizeof FB); }

void bsp_motor_write(uint32_t compare, bool positive) { FB.motor_compare = compare; FB.motor_positive = positive; FB.motor_writes++; }
void bsp_servo_write(uint8_t ch, uint16_t us) { if (ch < 2) FB.servo_us[ch] = us; }
void bsp_led_write(uint8_t mask) { FB.led_mask = mask; }
int32_t bsp_encoder_read(void) { return FB.encoder; }
uint8_t bsp_endstops_read(void) { return FB.endstops_raw; }
uint32_t bsp_micros(void) { return FB.now_ms * 1000u; }
uint32_t bsp_millis(void) { return FB.now_ms; }
void bsp_uart_send(const uint8_t *d, uint16_t n)
{
    if (FB.tx_len + n <= sizeof FB.tx) { memcpy(&FB.tx[FB.tx_len], d, n); FB.tx_len += n; }
}
