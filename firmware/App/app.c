/*
 * app.c - I/O bridge: applies MATLAB commands, reports encoder + endstops,
 * enforces the safety rules. All control loops run in MATLAB
 * (docs/decisions/0001-control-loop-in-matlab.md).
 *
 * Execution context:
 *   app_uart_rx_byte()  UART RX interrupt: only pushes into a ring buffer.
 *   app_poll()          main loop: parses frames, replies, applies outputs,
 *                       re-checks endstops and the watchdog every call.
 */
#include "app.h"
#include "config.h"
#include "bsp.h"
#include "motor/motor.h"
#include "servo/servo.h"

static app_t A;

/* single-producer (ISR) / single-consumer (main loop) ring buffer */
static volatile uint8_t  rx_buf[RX_RING_SIZE];
static volatile uint16_t rx_head;   /* written by the ISR */
static volatile uint16_t rx_tail;   /* written by the main loop */
static volatile uint32_t rx_overflow;

void app_uart_rx_byte(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_SIZE);
    if (next == rx_tail) { rx_overflow++; return; }   /* full: drop */
    rx_buf[rx_head] = byte;
    rx_head = next;
}

static bool rx_pop(uint8_t *b)
{
    if (rx_tail == rx_head) return false;
    *b = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_SIZE);
    return true;
}

static void outputs_safe(void)
{
    A.duty = 0; A.led_mask = 0;
    motor_apply(0);
    bsp_led_write(0);
}

static void apply_motor(uint8_t pressed)
{
    bool limited;
    int32_t d = safety_gate_duty(A.duty, pressed, &limited);
    A.limited = limited;
    motor_apply(d);
}

static void send_state(uint8_t seq, uint8_t pressed)
{
    uint8_t buf[PROTO_MAX_FRAME];
    uint32_t errs = A.parser.errors + rx_overflow;
    proto_state_t s;
    s.count = bsp_encoder_read();
    s.flags = pressed;
    if (A.safety.trip_event)        s.flags |= PROTO_F_WDOG;
    if (A.limited)                  s.flags |= PROTO_F_LIMITED;
    if (errs != A.rx_err_seen)      s.flags |= PROTO_F_RX_ERR;
    s.seq = seq;
    s.t_us = bsp_micros();
    s.rx_errors = (uint16_t)errs;
    A.safety.trip_event = false;
    A.rx_err_seen = errs;
    bsp_uart_send(buf, proto_encode_state(&s, buf));
}

static void handle(const proto_msg_t *m)
{
    uint8_t buf[PROTO_MAX_FRAME];
    proto_cmd_t c;
    uint8_t seq;
    uint8_t pressed = safety_endstops_pressed(bsp_endstops_read());

    if (proto_decode_cmd(m, &c)) {
        safety_cmd_received(&A.safety, bsp_millis());
        A.duty = motor_clamp(c.duty);
        A.servo_us[0] = servo_clamp(c.servo_us[0]);
        A.servo_us[1] = servo_clamp(c.servo_us[1]);
        A.led_mask = (uint8_t)(c.led_mask & ((1u << NUM_LEDS) - 1u));
        apply_motor(pressed);
        servo_apply(0, A.servo_us[0]);
        servo_apply(1, A.servo_us[1]);
        bsp_led_write(A.led_mask);
        A.frames_ok++;
        send_state(c.seq, pressed);
    } else if (m->type == PROTO_T_STOP && proto_decode_seq(m, &seq)) {
        outputs_safe();                       /* servos keep their position */
        A.limited = false;
        A.frames_ok++;
        send_state(seq, pressed);
    } else if (m->type == PROTO_T_HELLO && proto_decode_seq(m, &seq)) {
        A.frames_ok++;
        bsp_uart_send(buf, proto_encode_hello_reply(seq, bsp_micros(), buf));
    } else {
        A.parser.errors++;                    /* valid frame, unknown type / length */
    }
}

void app_init(void)
{
    proto_parser_init(&A.parser);
    safety_init(&A.safety);
    rx_head = rx_tail = 0; rx_overflow = 0;
    A.rx_err_seen = 0; A.frames_ok = 0; A.limited = false;
    A.servo_us[0] = A.servo_us[1] = SERVO_DEFAULT_US;
    outputs_safe();
    servo_apply(0, A.servo_us[0]);
    servo_apply(1, A.servo_us[1]);
}

void app_poll(void)
{
    uint8_t b;
    while (rx_pop(&b)) {
        if (proto_parser_feed(&A.parser, b)) handle(&A.parser.msg);
    }
    if (safety_watchdog(&A.safety, bsp_millis())) {
        if (A.duty != 0 || A.led_mask != 0) outputs_safe();
    }
    /* keep enforcing the endstops between commands */
    apply_motor(safety_endstops_pressed(bsp_endstops_read()));
}

const app_t *app_state(void) { return &A; }
