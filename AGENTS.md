# Repository instructions

This repository contains MATLAB eye-gaze software and STM32 NUCLEO-H753ZI C firmware. It currently contains documentation placeholders only.

- Inspect existing files and relevant documentation; preserve user work.
- Keep changes within the requested scope and MATLAB and firmware code separate.
- Mark unknowns as TBD; record approved decisions in docs/decisions/.
- Do not invent wiring, pins, clocks, peripherals, transport, protocol fields, units, timing, control gains, or electrical/PWM limits.
- Changes to architecture, safety behavior, and hardware limits require user authorization for that change. Existing authorization is sufficient for work within scope.
- Use interface/protocol.md as the shared contract once approved; keep both products and shared vectors consistent.
- Keep handwritten firmware logic in firmware/App/. Preserve generated code and user-code regions. Do not fabricate .ioc files or vendor drivers.
- Add meaningful tests for behavior changes. Check documentation-only scaffolding structurally.
- Report checks, unavailable tools, and limitations. Never claim hardware tests passed without recorded evidence.
- Do not commit, push, flash hardware, or operate actuators without user authorization.
- Do not commit secrets, personal eye recordings, or private calibration data. Document provenance of synthetic or consented fixtures.

Workflow: inspect, describe intended changes, implement the authorized task, perform available checks, and report results.
