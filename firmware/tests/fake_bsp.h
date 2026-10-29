#ifndef FAKE_BSP_H
#define FAKE_BSP_H
#include "../App/bsp.h"
typedef struct {
    uint32_t motor_compare; bool motor_positive; uint32_t motor_writes;
    uint16_t servo_us[2]; uint8_t led_mask;
    int32_t  encoder; uint8_t endstops_raw; uint32_t now_ms;
    uint8_t  tx[512]; uint16_t tx_len;
} fake_bsp_t;
extern fake_bsp_t FB;
void fake_reset(void);
#endif
