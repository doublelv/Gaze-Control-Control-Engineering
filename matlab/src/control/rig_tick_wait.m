function ok = rig_tick_wait(tk, k)
%RIG_TICK_WAIT  Real-time pacing for run_rig (serial mode only).
%   tk = struct('t0', tic, 'rt', realtime, 'Ts', Ts). Busy-waits until k*Ts after
%   tk.t0. Returns false if the period overran by more than one sample.
ok = true;
if ~tk.rt, return, end
if toc(tk.t0) > (k + 1)*tk.Ts, ok = false; end
while toc(tk.t0) < k*tk.Ts, end
end
