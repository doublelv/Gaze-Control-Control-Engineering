function [io, fb] = rig_stop(io)
%RIG_STOP  STOP frame: motor 0, LEDs off, servos keep their position.
C = proto_const();
io.seq = uint8(mod(double(io.seq) + 1, 256));
[io, raw] = transact(io, encode_simple(C.T_STOP, io.seq));
fb = decode_state(raw, io.seq);
end
