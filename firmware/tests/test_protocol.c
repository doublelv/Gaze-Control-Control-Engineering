/*
 * test_protocol.c - firmware codec against interface/test-vectors/protocol_v1.txt
 * (the same file the MATLAB tests use).
 */
#include "../App/communication/protocol.h"
#include "testutil.h"
#include <stdlib.h>
#include <string.h>

static long field(const char *fields, const char *key)
{
    char pat[32]; snprintf(pat, sizeof pat, "%s=", key);
    const char *p = fields;
    while ((p = strstr(p, pat)) != NULL) {
        if (p == fields || p[-1] == ' ') return strtol(p + strlen(pat), NULL, 10);
        p++;
    }
    printf("  missing field %s\n", key); g_fail++;
    return 0;
}
static unsigned long ufield(const char *f, const char *k) { return (unsigned long)strtoul(strstr(f, k) + strlen(k) + 1, NULL, 10); }

static int parse_all(const uint8_t *b, int n, proto_parser_t *p, int *nframes)
{
    *nframes = 0;
    for (int i = 0; i < n; i++) if (proto_parser_feed(p, b[i])) (*nframes)++;
    return *nframes;
}

void test_protocol(const char *path)
{
    FILE *f = fopen(path, "r");
    CHECK(f != NULL, "cannot open %s", path);
    if (!f) return;
    char line[512];
    int nvec = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        char *col[4] = {0}; int k = 0;
        for (char *t = strtok(line, "|"); t && k < 4; t = strtok(NULL, "|")) col[k++] = t;
        if (k < 4) continue;
        char name[32], kind[32]; sscanf(col[0], "%31s", name); sscanf(col[1], "%31s", kind);
        uint8_t b[64]; int n = 0; unsigned v;
        for (char *t = strtok(col[3], " \n"); t; t = strtok(NULL, " \n")) { sscanf(t, "%x", &v); b[n++] = (uint8_t)v; }
        const char *fl = col[2];
        nvec++;
        proto_parser_t p; proto_parser_init(&p); int nf;
        uint8_t out[PROTO_MAX_FRAME]; uint16_t len;
        if (!strcmp(kind, "BAD")) {
            parse_all(b, n, &p, &nf);
            CHECK(nf == 0, "%s: invalid frame accepted", name);
            continue;
        }
        parse_all(b, n, &p, &nf);
        CHECK(nf == 1, "%s: valid frame not parsed", name);
        if (!strcmp(kind, "CMD")) {
            proto_cmd_t c;
            CHECK(proto_decode_cmd(&p.msg, &c), "%s: decode", name);
            CHECK(c.duty == field(fl, "duty") && c.servo_us[0] == field(fl, "servoA") &&
                  c.servo_us[1] == field(fl, "servoB") && c.led_mask == field(fl, "led") &&
                  c.seq == field(fl, "seq"), "%s: fields", name);
            len = proto_encode_cmd(&c, out);
            CHECK(len == n && !memcmp(out, b, n), "%s: re-encode", name);
        } else if (!strcmp(kind, "HELLO") || !strcmp(kind, "STOP")) {
            uint8_t seq = 0;
            CHECK(proto_decode_seq(&p.msg, &seq) && seq == field(fl, "seq"), "%s: seq", name);
            CHECK(p.msg.type == (!strcmp(kind, "HELLO") ? PROTO_T_HELLO : PROTO_T_STOP), "%s: type", name);
        } else if (!strcmp(kind, "STATE")) {
            proto_state_t s;
            s.count = (int32_t)field(fl, "count"); s.flags = (uint8_t)field(fl, "flags");
            s.seq = (uint8_t)field(fl, "seq"); s.t_us = (uint32_t)ufield(fl, "t_us");
            s.rx_errors = (uint16_t)field(fl, "rx_err");
            len = proto_encode_state(&s, out);
            CHECK(len == n && !memcmp(out, b, n), "%s: encode", name);
        } else if (!strcmp(kind, "HELLO_REPLY")) {
            CHECK(field(fl, "proto") == PROTO_VERSION, "%s: version", name);
            len = proto_encode_hello_reply((uint8_t)field(fl, "seq"), (uint32_t)ufield(fl, "t_us"), out);
            CHECK(len == n && !memcmp(out, b, n), "%s: encode (fw version in config.h must match the vector)", name);
        } else {
            CHECK(0, "unknown kind %s", kind);
        }
    }
    fclose(f);
    CHECK(nvec >= 12, "only %d vectors", nvec);

    /* resync: garbage + AA AA 55 + valid frame */
    proto_parser_t p; proto_parser_init(&p);
    const uint8_t g[] = { 0x00, 0x13, 0xAA, 0xAA, 0x55, 0x02, 0x01, 0x05, 0x08 };
    int nf; parse_all(g, sizeof g, &p, &nf);
    CHECK(nf == 1 && p.msg.type == PROTO_T_HELLO, "resync after garbage");
    printf("test_protocol: %d vectors\n", nvec);
}
