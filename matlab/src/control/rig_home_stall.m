function [io, fb, ok] = rig_home_stall(io, P, sgn, tk)
%RIG_HOME_STALL  Drive slowly into a mechanical stop until the encoder stops.
%   sgn : -1 = toward MIN, +1 = toward MAX.  Duty P.home.duty, timeout 20 s.
%   Stall = count unchanged (+-1) for P.home.tStall after at least 0.3 s.
%   fb  : STATE after the motor was switched off (count at the stop).
cmd = struct('duty', sgn*P.home.duty*P.drv.pwmCounts, 'servoUs', P.srv.zeroUs, 'led', false(1, P.led.n));
ok = false; nStill = 0; lastCnt = NaN; tk.t0 = tic;
for k = 1:round(20/P.Ts)
    [io, fb] = rig_exchange(io, cmd);
    if fb.ok
        if ~isnan(lastCnt) && abs(double(fb.count) - lastCnt) <= 1, nStill = nStill + 1; else, nStill = 0; end
        lastCnt = double(fb.count);
        if k*P.Ts > 0.3 && nStill >= round(P.home.tStall/P.Ts), ok = true; break, end
    end
    rig_tick_wait(tk, k);
end
cmd.duty = 0; [io, fb] = rig_exchange(io, cmd);
end
