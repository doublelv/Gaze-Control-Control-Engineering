# .github/workflows

`ci.yml` runs on every push and pull request:
- `firmware`: `make -C firmware/tests` (gcc host tests + BSP syntax check).
- `matlab-octave`: `matlab/tests/run_all_tests` in GNU Octave (no MATLAB licence needed; synthetic
  data only, takes ~4 min).
No hardware is used in CI.
