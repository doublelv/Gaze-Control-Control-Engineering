/*
 * protocol.h - MATLAB <-> STM32 protocol v1 (interface/protocol.md).
 * Frame: AA 55 | type | len | payload[len] | checksum
 * checksum = (type + len + sum(payload)) mod 256. Little-endian fields.
 * Portable C99, no HAL dependency (host-tested in firmware/tests).
 */
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H
#include <stdint.h>
#include <stdbool.h>

#define PROTO_VERSION        1u
#define PROTO_HDR0           0xAAu
#define PROTO_HDR1           0x55u
#define PROTO_MAX_PAYLOAD    16u
#define PROTO_MAX_FRAME      (PROTO_MAX_PAYLOAD + 5u)

enum {
    PROTO_T_CMD         = 0x01,
    PROTO_T_HELLO       = 0x02,
    PROTO_T_STOP        = 0x03,
    PROTO_T_STATE       = 0x81,
    PROTO_T_HELLO_REPLY = 0x82
};

/* STATE flags */
#define PROTO_F_END_MIN   0x01u
#define PROTO_F_END_MAX   0x02u
#define PROTO_F_WDOG      0x04u
#define PROTO_F_LIMITED   0x08u
#define PROTO_F_RX_ERR    0x10u

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_msg_t;

typedef struct {
    int16_t  duty;
    uint16_t servo_us[2];
    uint8_t  led_mask;
    uint8_t  seq;
} proto_cmd_t;

typedef struct {
    int32_t  count;
    uint8_t  flags;
    uint8_t  seq;
    uint32_t t_us;
    uint16_t rx_errors;
} proto_state_t;

typedef struct {
    uint8_t  state;
    uint8_t  idx;
    uint8_t  sum;
    proto_msg_t msg;
    uint32_t errors;     /* malformed frames (bad length or checksum) */
} proto_parser_t;

void     proto_parser_init(proto_parser_t *p);
/* Feed one byte. Returns true when a complete valid frame is in p->msg. */
bool     proto_parser_feed(proto_parser_t *p, uint8_t byte);

bool     proto_decode_cmd(const proto_msg_t *m, proto_cmd_t *out);
bool     proto_decode_seq(const proto_msg_t *m, uint8_t *seq);   /* HELLO / STOP */

/* Encoders: write a complete frame into buf, return its length. */
uint16_t proto_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *buf);
uint16_t proto_encode_state(const proto_state_t *s, uint8_t *buf);
uint16_t proto_encode_hello_reply(uint8_t seq, uint32_t t_us, uint8_t *buf);
uint16_t proto_encode_cmd(const proto_cmd_t *c, uint8_t *buf);    /* host tools / tests */

#endif
