function [io, y] = rig_move_to(io, target, yOf, C, M, P, tk)
%RIG_MOVE_TO  Closed-loop move of the carriage to an output-shaft angle [rad]
%   (used by run_rig TASK = 'sysid'). yOf: function fb -> angle. Moves at half
%   the maximum slew rate, stops within 1 mm or after 4 s.
cmd = struct('duty', 0, 'servoUs', P.srv.zeroUs, 'led', false(1, P.led.n));
[io, fb] = rig_exchange(io, cmd); y = yOf(fb);
st = struct('I', C.Kp*(1 - P.bSP)*y, 'D', 0, 'yPrev', y);
ref = y; tk.t0 = tic;
for k = 1:round(4/P.Ts)
    ref = ref + min(max(target - ref, -0.5*P.ax(1).vmax*P.Ts), 0.5*P.ax(1).vmax*P.Ts);
    [V, st] = pid_step(st, C, M, P, ref, y);
    cmd.duty = V/M.Vmax*P.drv.pwmCounts;
    [io, fb] = rig_exchange(io, cmd); if fb.ok, y = yOf(fb); end
    rig_tick_wait(tk, k);
    if abs(target - y)*M.r < 1e-3 && abs(target - ref) < eps, break, end
end
cmd.duty = 0; [io, ~] = rig_exchange(io, cmd);
end
