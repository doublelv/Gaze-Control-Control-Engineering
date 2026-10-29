# 0003 - MATLAB/STM32 protocol v1 (framed, typed messages)

- Date: 2026-10-29
- Status: **accepted**

## Context
v0 was a fixed 11/13-byte exchange without message types, version or error reporting.

## Decision
Frames `AA 55 | type | len | payload | checksum`, message types CMD / HELLO / STOP and
STATE / HELLO_REPLY, a status-flag byte (endstops, watchdog trip, endstop limiting, receive
errors) and an error counter. Details: `interface/protocol.md`.

## Rationale
- HELLO gives a clear error for a wrong COM port, unflashed board or version mismatch.
- The length byte allows resynchronisation and future messages without breaking old ones.
- The flags make firmware safety actions visible in MATLAB logs.

## Consequences
Both implementations are tested against `interface/test-vectors/protocol_v1.txt`
(`matlab/tests/test_protocol.m`, `firmware/tests/test_protocol.c`).
