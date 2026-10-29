function C = design_pid(M, ax, Ts)
%DESIGN_PID  PID by pole placement on the reduced plant theta'' + a theta' = k V.
%   Closed loop: s^3 + (a + k Kd) s^2 + k Kp s + k Ki
%   Target:     (s + alpha wn)(s^2 + 2 zeta wn s + wn^2)

wn = 2*pi*ax.bwHz;
z  = ax.zeta;
al = ax.alpha;

C.Kd = ((2*z + al)*wn - M.a)/M.k;
C.Kp = wn^2*(1 + 2*z*al)/M.k;
C.Ki = al*wn^3/M.k;

if C.Kd < 0
    warning('%s: required Kd < 0 (plant already damped enough). Kd set to 0; poles will differ from target.', M.name);
    C.Kd = 0;
end
if ax.bwHz > 1/(20*Ts)
    warning('%s: bandwidth %.1f Hz is high for Ts = %.2g s (rule of thumb fs > 20*bw).', ...
            M.name, ax.bwHz, Ts);
end

% Derivative low-pass filter time constant
C.Tf = max(1/(10*wn), 2*Ts);
C.wn = wn;
end
