/* safety.c - see safety.h */
#include "safety.h"
#include "../config.h"
#include "../communication/protocol.h"

void safety_init(safety_t *s)
{
    s->last_cmd_ms = 0; s->armed = false; s->tripped = false; s->trip_event = false;
}

void safety_cmd_received(safety_t *s, uint32_t now_ms)
{
    s->last_cmd_ms = now_ms; s->armed = true; s->tripped = false;
}

bool safety_watchdog(safety_t *s, uint32_t now_ms)
{
    if (s->armed && !s->tripped && (uint32_t)(now_ms - s->last_cmd_ms) > WATCHDOG_MS) {
        s->tripped = true; s->trip_event = true;
    }
    return s->tripped;
}

uint8_t safety_endstops_pressed(uint8_t raw)
{
    uint8_t lvl = ENDSTOP_ACTIVE_HIGH ? raw : (uint8_t)~raw;
    uint8_t out = 0;
#if USE_END_MIN
    if (lvl & 0x01u) out |= PROTO_F_END_MIN;
#endif
#if USE_END_MAX
    if (lvl & 0x02u) out |= PROTO_F_END_MAX;
#endif
    return out;
}

int32_t safety_gate_duty(int32_t duty, uint8_t pressed, bool *limited)
{
    *limited = false;
    if (((pressed & PROTO_F_END_MIN) && duty < 0) || ((pressed & PROTO_F_END_MAX) && duty > 0)) {
        *limited = true;
        return 0;
    }
    return duty;
}
