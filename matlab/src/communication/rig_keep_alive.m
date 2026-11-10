function [io, tTx, msg] = rig_keep_alive(io, cmd, tk, tTx, realtime, P)
%RIG_KEEP_ALIVE  Re-send the last command during long camera/detection/drawing steps.
%   The firmware stops the motor if no command arrives for 100 ms (watchdog). With
%   the camera a loop can take 100-400 ms, and the motor then stopped in the middle
%   of a move (2026-11-10: 299 watchdog stops in 157 s, settling 1.1-2.1 s instead
%   of < 1 s). Re-sending the SAME voltage keeps it applied for the whole loop, as
%   the controller design assumes. If MATLAB hangs, nothing is sent and the
%   watchdog still stops the motor.
msg = '';
if ~realtime || toc(tk.t0) - tTx < P.link.keepAlive, return, end
try
    io = rig_exchange(io, cmd);
catch err
    msg = err.message;
end
tTx = toc(tk.t0);
end
