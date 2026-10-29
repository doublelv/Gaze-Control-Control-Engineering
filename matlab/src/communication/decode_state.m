function fb = decode_state(raw, seq)
%DECODE_STATE  STATE frame (MCU -> PC), payload 12 bytes:
%   int32 count | uint8 flags | uint8 seq | uint32 t_us | uint16 rxErrors
%   fb.ok = false for a malformed frame, another type, or a wrong seq.
C = proto_const();
fb = struct('count', int32(0), 'flags', uint8(0), 'endMin', false, 'endMax', false, ...
            'wdog', false, 'limited', false, 'rxErr', false, 'seq', uint8(0), ...
            'tUs', 0, 'rxErrors', 0, 'ok', false);
[msg, ok] = proto_parse(raw);
if ~ok || msg.type ~= C.T_STATE || numel(msg.payload) ~= 12, return, end
p = msg.payload;
fb.count    = typecast(p(1:4), 'int32');
fb.flags    = p(5);
fb.endMin   = bitand(p(5), C.F_END_MIN) > 0;
fb.endMax   = bitand(p(5), C.F_END_MAX) > 0;
fb.wdog     = bitand(p(5), C.F_WDOG) > 0;
fb.limited  = bitand(p(5), C.F_LIMITED) > 0;
fb.rxErr    = bitand(p(5), C.F_RX_ERR) > 0;
fb.seq      = p(6);
fb.tUs      = double(typecast(p(7:10), 'uint32'));
fb.rxErrors = double(typecast(p(11:12), 'uint16'));
fb.ok       = nargin < 2 || isempty(seq) || fb.seq == uint8(seq);
end
