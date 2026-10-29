function [msg, ok] = proto_parse(raw)
%PROTO_PARSE  Check one complete frame and return its type and payload.
%   ok = false for wrong header, length or checksum.
C = proto_const();
raw = uint8(raw(:)');
msg = struct('type', uint8(0), 'payload', uint8([]));
ok = false;
if numel(raw) < 5 || ~isequal(raw(1:2), C.header), return, end
len = double(raw(4));
if len > C.maxLen || numel(raw) ~= len + 5, return, end
if mod(sum(double(raw(3:end-1))), 256) ~= raw(end), return, end
msg.type = raw(3);
msg.payload = raw(5:4+len);
ok = true;
end
