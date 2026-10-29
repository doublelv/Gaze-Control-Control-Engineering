# matlab/scripts

Entry points (see `../README.md` for the table and the recommended order).
`setup_paths.m` adds `src/*`, `config`, `config/local` to the path (no hardware effects).
Only `run_rig` with `MODE='serial'` moves actuators; it asks for confirmation before the
io_check motor pulse and always stops the motor on exit.
