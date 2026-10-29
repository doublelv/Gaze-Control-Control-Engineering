/*
 * safety.h - firmware-side safety rules, independent of MATLAB:
 *  - endstop gating: no motion further into a pressed endstop
 *  - watchdog: motor 0 + LEDs off if no valid CMD for WATCHDOG_MS
 */
#ifndef APP_SAFETY_H
#define APP_SAFETY_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t last_cmd_ms;
    bool     armed;        /* becomes true with the first CMD */
    bool     tripped;      /* currently in watchdog stop */
    bool     trip_event;   /* tripped since the last STATE (reported once) */
} safety_t;

void    safety_init(safety_t *s);
void    safety_cmd_received(safety_t *s, uint32_t now_ms);
/* Returns true while the watchdog stop is active. */
bool    safety_watchdog(safety_t *s, uint32_t now_ms);
/* Endstop levels (raw) -> pressed bits (PROTO_F_END_MIN / _MAX), respecting config. */
uint8_t safety_endstops_pressed(uint8_t raw_levels);
/* Duty after endstop gating; *limited = true if it was forced to 0. */
int32_t safety_gate_duty(int32_t duty, uint8_t pressed, bool *limited);

#endif
