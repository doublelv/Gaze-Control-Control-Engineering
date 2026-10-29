function pkt = proto_frame(type, payload)
%PROTO_FRAME  Build a frame: AA 55 | type | len | payload | checksum.
%   checksum = (type + len + sum(payload)) mod 256
C = proto_const();
payload = uint8(payload(:)');
assert(numel(payload) <= C.maxLen, 'payload too long');
body = [uint8(type), uint8(numel(payload)), payload];
pkt  = [C.header, body, uint8(mod(sum(double(body)), 256))];
end
