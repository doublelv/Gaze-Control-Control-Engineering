function [io, raw] = transact(io, pkt)
%TRANSACT  Send one frame and read one reply frame (raw bytes, [] on timeout).
if strcmp(io.mode, 'virtual')
    [io.sim, raw] = virtual_mcu_step(io.sim, pkt, io.P);
    return
end
write(io.s, pkt, 'uint8');
raw = [];
hdr = read(io.s, 4, 'uint8');
if numel(hdr) < 4
    io.nTimeout = io.nTimeout + 1; return
end
C = proto_const();
if hdr(1) ~= 170 || hdr(2) ~= 85 || hdr(4) > C.maxLen
    flush(io.s); return                            % out of sync: drop and resync
end
rest = read(io.s, double(hdr(4)) + 1, 'uint8');
if numel(rest) < double(hdr(4)) + 1
    io.nTimeout = io.nTimeout + 1; return
end
raw = uint8([hdr(:)' rest(:)']);
end
