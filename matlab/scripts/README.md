# matlab/scripts

Entry points. The table with the status marks (RIG TOOL LEDS ANALYSIS, see `../README.md`) and the order on the
hardware are in `../README.md`.

- `setup_paths.m` adds `src/*`, `config`, `config/local` to the path (no hardware effects).
- Scripts that move hardware: `run_rig` with `MODE='serial'` (motor + servos), `motor_test` (motor),
  `servo_test` (servos only, motor off). `run_rig` always stops the motor on exit and asks before the
  io_check motor pulse; the firmware watchdog stops the motor 100 ms after MATLAB stops sending.
- Logs: `run_rig` saves `artifacts/logs/rig_<task>_<date>.mat` (L, P, C); `sysid` saves
  `artifacts/sysid_<date>.mat`.
