# interface/test-vectors

`protocol_v1.txt`: one vector per line, `name | KIND | fields | hex bytes`. KIND = CMD, HELLO, STOP,
STATE, HELLO_REPLY or BAD (must be rejected). All values are synthetic.
Regenerate only on purpose with `tools/make_test_vectors.m`; then both
`matlab/tests/run_all_tests` and `make -C firmware/tests` must pass.
