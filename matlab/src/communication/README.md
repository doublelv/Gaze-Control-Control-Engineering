# matlab/src/communication

Protocol v1 (`interface/protocol.md`).

| File | Purpose |
|---|---|
| `proto_const.m`, `proto_frame.m`, `proto_parse.m` | Framing, types, flags, checksum |
| `encode_command.m`, `encode_simple.m`, `encode_state.m` | Encoders (CMD, HELLO/STOP, STATE for the virtual MCU and tests) |
| `decode_state.m`, `decode_hello.m` | Decoders with seq check |
| `rig_open.m` | Opens `serial` (checks the port exists, HELLO + version) or `virtual` |
| `rig_exchange.m`, `transact.m` | One CMD -> STATE round trip, resync, error counting |
| `rig_stop.m`, `rig_close.m` | STOP frame, release the port |
| `virtual_mcu_init.m`, `virtual_mcu_step.m` | Simulated firmware + motor/belt speaking the same bytes (watchdog, endstop gating, flags) |
