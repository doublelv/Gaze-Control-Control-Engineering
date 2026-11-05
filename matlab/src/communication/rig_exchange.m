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
    % encoder plausibility: a jump faster than the motor can move means the Nucleo
    % restarted (counter reset, e.g. a supply dip from the servos) or a bad reading
    if isfield(io, 'encLast') && ~isempty(io.encLast)
        if strcmp(io.mode, 'serial'), dt = max(toc(io.encT), io.P.Ts); else, dt = io.P.Ts; end
        jump = abs(double(fb.count) - io.encLast);
        if jump > io.P.link.maxCountRate*dt + 300
            try, rig_stop(io); catch, end
            error('rig:encoderJump', ['Encoder jumped %d counts in %.0f ms -> motor STOPPED. ', ...
                  'Did the Nucleo reset (servo powered from the Nucleo 5V?) or the encoder cable move? ', ...
                  'Press RESET, then run again (homing is redone).'], round(jump), dt*1e3);
        end
    end
    io.encLast = double(fb.count); io.encT = tic;
    io.last = fb;
end
end
