/* protocol.c - see protocol.h */
#include "protocol.h"
#include "../config.h"
#include <string.h>

enum { S_HDR0, S_HDR1, S_TYPE, S_LEN, S_PAYLOAD, S_CKS };

static void put_u16(uint8_t *b, uint16_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *b) { return (uint16_t)(b[0] | ((uint16_t)b[1] << 8)); }

void proto_parser_init(proto_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = S_HDR0;
}

bool proto_parser_feed(proto_parser_t *p, uint8_t byte)
{
    switch (p->state) {
    case S_HDR0:
        if (byte == PROTO_HDR0) p->state = S_HDR1;
        break;
    case S_HDR1:
        if (byte == PROTO_HDR1)      p->state = S_TYPE;
        else if (byte != PROTO_HDR0) p->state = S_HDR0;   /* AA AA 55 still syncs */
        break;
    case S_TYPE:
        p->msg.type = byte; p->sum = byte; p->state = S_LEN;
        break;
    case S_LEN:
        if (byte > PROTO_MAX_PAYLOAD) { p->errors++; p->state = S_HDR0; break; }
        p->msg.len = byte; p->sum = (uint8_t)(p->sum + byte); p->idx = 0;
        p->state = (byte == 0) ? S_CKS : S_PAYLOAD;
        break;
    case S_PAYLOAD:
        p->msg.payload[p->idx++] = byte; p->sum = (uint8_t)(p->sum + byte);
        if (p->idx >= p->msg.len) p->state = S_CKS;
        break;
    case S_CKS:
        p->state = S_HDR0;
        if (byte == p->sum) return true;
        p->errors++;
        break;
    default:
        p->state = S_HDR0;
        break;
    }
    return false;
}

bool proto_decode_cmd(const proto_msg_t *m, proto_cmd_t *out)
{
    if (m->type != PROTO_T_CMD || m->len != 8u) return false;
    out->duty        = (int16_t)get_u16(&m->payload[0]);
    out->servo_us[0] = get_u16(&m->payload[2]);
    out->servo_us[1] = get_u16(&m->payload[4]);
    out->led_mask    = m->payload[6];
    out->seq         = m->payload[7];
    return true;
}

bool proto_decode_seq(const proto_msg_t *m, uint8_t *seq)
{
    if ((m->type != PROTO_T_HELLO && m->type != PROTO_T_STOP) || m->len != 1u) return false;
    *seq = m->payload[0];
    return true;
}

uint16_t proto_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *buf)
{
    uint8_t sum = (uint8_t)(type + len);
    buf[0] = PROTO_HDR0; buf[1] = PROTO_HDR1; buf[2] = type; buf[3] = len;
    for (uint8_t i = 0; i < len; i++) { buf[4u + i] = payload[i]; sum = (uint8_t)(sum + payload[i]); }
    buf[4u + len] = sum;
    return (uint16_t)(len + 5u);
}

uint16_t proto_encode_state(const proto_state_t *s, uint8_t *buf)
{
    uint8_t p[12];
    put_u32(&p[0], (uint32_t)s->count);
    p[4] = s->flags; p[5] = s->seq;
    put_u32(&p[6], s->t_us);
    put_u16(&p[10], s->rx_errors);
    return proto_frame(PROTO_T_STATE, p, sizeof p, buf);
}

uint16_t proto_encode_hello_reply(uint8_t seq, uint32_t t_us, uint8_t *buf)
{
    uint8_t p[8] = { PROTO_VERSION, FW_VERSION_MAJOR, FW_VERSION_MINOR, seq, 0, 0, 0, 0 };
    put_u32(&p[4], t_us);
    return proto_frame(PROTO_T_HELLO_REPLY, p, sizeof p, buf);
}

uint16_t proto_encode_cmd(const proto_cmd_t *c, uint8_t *buf)
{
    uint8_t p[8];
    put_u16(&p[0], (uint16_t)c->duty);
    put_u16(&p[2], c->servo_us[0]);
    put_u16(&p[4], c->servo_us[1]);
    p[6] = c->led_mask; p[7] = c->seq;
    return proto_frame(PROTO_T_CMD, p, sizeof p, buf);
}
