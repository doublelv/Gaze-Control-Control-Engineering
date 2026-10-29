# MATLAB / STM32 interface contract

Status: TBD / not approved. No transport or message format is selected. Both products must use this shared contract once agreed.

Define and document:
- Control-loop ownership and command/feedback responsibilities.
- Transport, startup, connection and reconnection lifecycle.
- Versioning and compatibility.
- Coordinate frames, axis directions, units, ranges, and calibration ownership.
- Message types, fields, data types, representation, framing, length constraints, and byte order where applicable.
- Update rates, timestamps, stale-data rules, and timeouts.
- Validation, error detection, malformed-input handling, and buffer limits.
- Acknowledgement, ordering, duplication, and retry rules where applicable.
- Approved startup, disconnect, fault, and recovery behavior linked to safety requirements.

Record rationale and requirement references. Add exact valid and invalid examples in test-vectors/ and run both implementations against them. Host tests and hardware integration results must be reported separately. No settings or safe-state behavior are implied by this placeholder.
