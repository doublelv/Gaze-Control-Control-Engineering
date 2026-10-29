function pkt = encode_simple(type, seq)
%ENCODE_SIMPLE  HELLO or STOP frame (payload = 1 byte seq).
pkt = proto_frame(type, uint8(seq));
end
