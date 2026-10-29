function test_protocol()
%TEST_PROTOCOL  MATLAB codec against interface/test-vectors/protocol_v1.txt:
%   every valid vector must encode to the same bytes and decode to the same
%   fields; every BAD vector must be rejected.
P = params(); C = proto_const();
V = read_vectors();
assert(numel(V) >= 12, 'too few vectors');
for v = V
    f = v.f; b = v.bytes;
    switch v.kind
        case 'CMD'
            cmd = struct('duty', f.duty, 'servoUs', [f.servoA f.servoB], 'led', bitget(f.led, 1:4));
            assert(isequal(encode_command(cmd, f.seq, P), b), '%s: encode', v.name);
            [m, ok] = proto_parse(b);
            assert(ok && m.type == C.T_CMD, '%s: parse', v.name);
        case {'HELLO', 'STOP'}
            t = C.T_HELLO; if strcmp(v.kind, 'STOP'), t = C.T_STOP; end
            assert(isequal(encode_simple(t, f.seq), b), '%s: encode', v.name);
        case 'STATE'
            fb = decode_state(b, f.seq);
            assert(fb.ok, '%s: decode', v.name);
            assert(double(fb.count) == f.count && double(fb.flags) == f.flags && ...
                   fb.tUs == f.t_us && fb.rxErrors == f.rx_err, '%s: fields', v.name);
            assert(isequal(encode_state(struct('count', f.count, 'flags', f.flags, 'seq', f.seq, ...
                   'tUs', f.t_us, 'rxErrors', f.rx_err)), b), '%s: re-encode', v.name);
            assert(~decode_state(b, mod(f.seq + 1, 256)).ok, '%s: wrong seq accepted', v.name);
        case 'HELLO_REPLY'
            h = decode_hello(b, f.seq);
            assert(h.ok && h.proto == f.proto && h.fwMajor == f.fw_major && ...
                   h.fwMinor == f.fw_minor && h.tUs == f.t_us, '%s: decode', v.name);
        case 'BAD'
            [~, ok] = proto_parse(b);
            assert(~ok, '%s: invalid frame accepted', v.name);
        otherwise
            error('unknown kind %s', v.kind);
    end
end
% clipping
cmd = struct('duty', 99999, 'servoUs', [100 9999], 'led', [1 0 0 0]);
[m, ok] = proto_parse(encode_command(cmd, 3, P));
assert(ok && typecast(m.payload(1:2), 'int16') == P.drv.pwmCounts, 'duty not clipped');
assert(typecast(m.payload(3:4), 'uint16') == P.srv.pulseMin && ...
       typecast(m.payload(5:6), 'uint16') == P.srv.pulseMax, 'servo not clipped');
end
