# firmware/tests

Host unit tests (gcc, no board): `make` here.
- `test_protocol.c`: parser/encoders against `interface/test-vectors/protocol_v1.txt` (same file as the MATLAB tests) + resync.
- `test_app.c`: application with a fake BSP: start-up state, CMD/STATE, clamping, endstop gating (also between commands), watchdog, corrupt frames, STOP, ring-buffer overflow.
- `make check-bsp`: syntax check of the STM32 BSP against `stubs/main.h`.
Hardware integration results are reported separately (none yet).
