# tools

| File | Purpose |
|---|---|
| `run_tests.sh` | All offline tests (firmware gcc + MATLAB in Octave), same as CI. Linux/macOS/WSL. |
| `make_test_vectors.m` | Regenerates `interface/test-vectors/protocol_v1.txt` from the MATLAB codec. Only after an intended protocol change. |

In MATLAB itself: `cd matlab/tests; run_all_tests`. Firmware only: `make -C firmware/tests`.
