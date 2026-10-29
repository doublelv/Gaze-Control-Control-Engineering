function pkt = encode_state(fb)
%ENCODE_STATE  Build a STATE frame (used by the virtual rig and the tests).
C = proto_const();
payload = [typecast(int32(fb.count), 'uint8'), uint8(fb.flags), uint8(fb.seq), ...
           typecast(uint32(fb.tUs), 'uint8'), typecast(uint16(fb.rxErrors), 'uint8')];
pkt = proto_frame(C.T_STATE, payload);
end
