/*
 * app.h - top level of the I/O-bridge firmware.
 * Integration in the CubeMX-generated main.c (see firmware/Core/Src/README.md):
 *   USER CODE 2      : app_init();
 *   USER CODE 3 loop : app_poll();
 *   UART RX callback : app_uart_rx_byte(byte);   (from the interrupt)
 */
#ifndef APP_APP_H
#define APP_APP_H
#include <stdint.h>
#include <stdbool.h>
#include "communication/protocol.h"
#include "safety/safety.h"

typedef struct {
    proto_parser_t parser;
    safety_t       safety;
    int32_t        duty;          /* last commanded duty (before gating) */
    uint16_t       servo_us[2];
    uint8_t        led_mask;
    bool           limited;       /* duty currently forced to 0 by an endstop */
    uint32_t       rx_err_seen;   /* parser errors already reported */
    uint32_t       frames_ok;
} app_t;

void   app_init(void);
void   app_poll(void);
void   app_uart_rx_byte(uint8_t byte);
const app_t *app_state(void);   /* for tests / debugging */

#endif
