/* test_app.c - application behaviour with the fake BSP. */
#include "../App/app.h"
#include "../App/config.h"
#include "fake_bsp.h"
#include "testutil.h"
#include <string.h>

static void send(const uint8_t *b, uint16_t n) { for (uint16_t i = 0; i < n; i++) app_uart_rx_byte(b[i]); app_poll(); }

static void send_cmd(int16_t duty, uint16_t a, uint16_t b, uint8_t led, uint8_t seq)
{
    proto_cmd_t c = { duty, { a, b }, led, seq };
    uint8_t buf[PROTO_MAX_FRAME];
    send(buf, proto_encode_cmd(&c, buf));
}

static int last_state(proto_state_t *s)
{
    proto_parser_t p; proto_parser_init(&p);
    int found = 0;
    for (uint16_t i = 0; i < FB.tx_len; i++) {
        if (proto_parser_feed(&p, FB.tx[i]) && p.msg.type == PROTO_T_STATE) {
            const uint8_t *q = p.msg.payload;
            s->count = (int32_t)((uint32_t)q[0] | (uint32_t)q[1] << 8 | (uint32_t)q[2] << 16 | (uint32_t)q[3] << 24);
            s->flags = q[4]; s->seq = q[5];
            s->rx_errors = (uint16_t)(q[10] | q[11] << 8);
            found = 1;
        }
    }
    FB.tx_len = 0;
    return found;
}

void test_app(void)
{
    proto_state_t s;
    fake_reset(); app_init();
    CHECK(FB.motor_compare == 0 && FB.led_mask == 0 && FB.servo_us[0] == SERVO_DEFAULT_US, "safe start-up state");

    /* HELLO -> HELLO_REPLY with version */
    uint8_t hello[] = { 0xAA, 0x55, 0x02, 0x01, 0x05, 0x08 };
    send(hello, sizeof hello);
    CHECK(FB.tx_len == 13 && FB.tx[2] == PROTO_T_HELLO_REPLY && FB.tx[4] == PROTO_VERSION && FB.tx[7] == 5, "hello reply");
    FB.tx_len = 0;

    /* CMD applies outputs and returns STATE with encoder + seq */
    FB.encoder = 4321; FB.now_ms = 10;
    send_cmd(6000, 1400, 1700, 0x0B, 7);
    CHECK(FB.motor_compare == 6000 && FB.motor_positive == (MOTOR_DIR_POSITIVE_HIGH != 0), "motor applied");
    CHECK(FB.servo_us[0] == 1400 && FB.servo_us[1] == 1700 && FB.led_mask == 0x0B, "servos + LEDs");
    CHECK(last_state(&s) && s.count == 4321 && s.seq == 7 && s.flags == 0, "state reply");

    /* clamping */
    send_cmd(-30000, 100, 9000, 0xFF, 8);
    CHECK(FB.motor_compare == MOTOR_PWM_COUNTS && FB.motor_positive == (MOTOR_DIR_POSITIVE_HIGH == 0), "duty clamp/dir");
    CHECK(FB.servo_us[0] == SERVO_MIN_US && FB.servo_us[1] == SERVO_MAX_US && FB.led_mask == 0x0F, "servo/LED clamp");
    last_state(&s);

    /* endstop MIN pressed: negative duty blocked, positive allowed, flags reported */
    FB.endstops_raw = ENDSTOP_ACTIVE_HIGH ? 0x01 : 0xFE;
    send_cmd(-5000, 1500, 1500, 0, 9);
    CHECK(FB.motor_compare == 0, "driving into MIN must be blocked");
    CHECK(last_state(&s) && (s.flags & PROTO_F_END_MIN) && (s.flags & PROTO_F_LIMITED), "endstop flags");
    send_cmd(5000, 1500, 1500, 0, 10);
    CHECK(FB.motor_compare == 5000, "driving away from MIN allowed");
    last_state(&s);
    /* endstop pressed between commands while moving toward it -> stop in app_poll */
    send_cmd(-5000, 1500, 1500, 0, 11); last_state(&s);
    FB.endstops_raw = ENDSTOP_ACTIVE_HIGH ? 0x00 : 0xFF;
    send_cmd(-5000, 1500, 1500, 0, 12); last_state(&s);
    CHECK(FB.motor_compare == 5000, "moving toward MIN while released");
    FB.endstops_raw = ENDSTOP_ACTIVE_HIGH ? 0x01 : 0xFE;
    app_poll();
    CHECK(FB.motor_compare == 0, "endstop enforced between commands");
    FB.endstops_raw = ENDSTOP_ACTIVE_HIGH ? 0x00 : 0xFF;

    /* watchdog */
    FB.now_ms = 1000; send_cmd(3000, 1500, 1500, 0x0F, 13); last_state(&s);
    FB.now_ms = 1000 + WATCHDOG_MS - 1; app_poll();
    CHECK(FB.motor_compare == 3000 && FB.led_mask == 0x0F, "no trip before timeout");
    FB.now_ms = 1000 + WATCHDOG_MS + 1; app_poll();
    CHECK(FB.motor_compare == 0 && FB.led_mask == 0, "watchdog stops motor + LEDs");
    CHECK(FB.servo_us[0] == 1500, "servos hold position");
    send_cmd(0, 1500, 1500, 0, 14);
    CHECK(last_state(&s) && (s.flags & PROTO_F_WDOG), "watchdog flag reported once");
    send_cmd(0, 1500, 1500, 0, 15);
    CHECK(last_state(&s) && !(s.flags & PROTO_F_WDOG), "watchdog flag cleared");

    /* corrupt frame -> no reply, error counted and flagged */
    uint8_t bad[] = { 0xAA, 0x55, 0x01, 0x08, 1, 2, 3, 4, 5, 6, 7, 8, 0x00 };
    send(bad, sizeof bad);
    CHECK(FB.tx_len == 0, "no reply to a corrupt frame");
    send_cmd(0, 1500, 1500, 0, 16);
    CHECK(last_state(&s) && (s.flags & PROTO_F_RX_ERR) && s.rx_errors == 1, "rx error flag + counter");

    /* STOP */
    send_cmd(4000, 1500, 1500, 0x0F, 17); last_state(&s);
    uint8_t stop[] = { 0xAA, 0x55, 0x03, 0x01, 0x12, 0x16 };
    send(stop, sizeof stop);
    CHECK(FB.motor_compare == 0 && FB.led_mask == 0 && last_state(&s) && s.seq == 0x12, "STOP");

    /* ring buffer overflow is counted, not a crash */
    for (int i = 0; i < 3 * (int)RX_RING_SIZE; i++) app_uart_rx_byte(0x00);
    app_poll();
    send_cmd(0, 1500, 1500, 0, 18);
    CHECK(last_state(&s) && (s.flags & PROTO_F_RX_ERR), "overflow reported");
    printf("test_app: done\n");
}
