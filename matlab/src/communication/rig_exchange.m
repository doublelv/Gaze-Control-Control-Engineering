function [io, fb] = rig_exchange(io, cmd)
%RIG_EXCHANGE  Send one CMD to the Nucleo and decode its STATE reply.
%   cmd.duty    : PWM counts [-pwmCounts, pwmCounts]
%   cmd.servoUs : [A B] pulse widths [us]
%   cmd.led     : 1 x nLED logical
%   fb          : see decode_state.m (fb.ok = false on timeout / corrupt reply)
io.seq = uint8(mod(double(io.seq) + 1, 256));
[io, raw] = transact(io, encode_command(cmd, io.seq, io.P));
fb = decode_state(raw, io.seq);
if ~fb.ok
    io.nErr = io.nErr + 1;
    if strcmp(io.mode, 'serial'), flush(io.s); end
else
    if fb.wdog, io.nWdog = io.nWdog + 1; end
    io.last = fb;
end
end
