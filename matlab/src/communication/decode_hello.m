function h = decode_hello(raw, seq)
%DECODE_HELLO  HELLO_REPLY frame, payload 8 bytes:
%   uint8 protoVersion | uint8 fwMajor | uint8 fwMinor | uint8 seq | uint32 t_us
C = proto_const();
h = struct('proto', 0, 'fwMajor', 0, 'fwMinor', 0, 'seq', uint8(0), 'tUs', 0, 'ok', false);
[msg, ok] = proto_parse(raw);
if ~ok || msg.type ~= C.T_HELLO_REPLY || numel(msg.payload) ~= 8, return, end
p = msg.payload;
h.proto = double(p(1)); h.fwMajor = double(p(2)); h.fwMinor = double(p(3));
h.seq = p(4); h.tUs = double(typecast(p(5:8), 'uint32'));
h.ok = nargin < 2 || isempty(seq) || h.seq == uint8(seq);
end
