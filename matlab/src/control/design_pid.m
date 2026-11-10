function C = design_pid(M, ax, Ts)
%DESIGN_PID  PID by pole placement on the reduced plant theta'' + a theta' = k V,
%   plus a phase-margin rule for the D gain.
%   Closed loop: s^3 + (a + k Kd) s^2 + k Kp s + k Ki
%   Target:     (s + alpha wn)(s^2 + 2 zeta wn s + wn^2)
%   Step 1 (pole placement) gives Kp, Ki and Kd. At 1 Hz the Kd from step 1 is
%   negative (the motor's back-EMF already damps more than the target needs),
%   so it is clipped to 0.
%   Step 2 (ax.pmTarget, ax.TsPM): the loop period on the rig is longer and
%   irregular, and the delay of 1.5 loop periods costs phase. The D term adds
%   phase lead near the crossover, so Kd is raised (in steps of 0.002) until the
%   phase margin at the loop period ax.TsPM reaches ax.pmTarget degrees.
%   Without these fields (or NaN) only step 1 is used (older designs).

wn = 2*pi*ax.bwHz;
z  = ax.zeta;
al = ax.alpha;

C.Kd = ((2*z + al)*wn - M.a)/M.k;
C.Kp = wn^2*(1 + 2*z*al)/M.k;
C.Ki = al*wn^3/M.k;
C.KdPolePlacement = C.Kd;

% Derivative low-pass filter time constant
C.Tf = max(1/(10*wn), 2*Ts);
C.wn = wn;

useLead = isfield(ax, 'pmTarget') && ~isempty(ax.pmTarget) && ~isnan(ax.pmTarget);
if C.Kd < 0
    if ~useLead
        warning('%s: required Kd < 0 (plant already damped enough). Kd set to 0; poles will differ from target.', M.name);
    end
    C.Kd = 0;
end
C.pmTarget = NaN; C.TsPM = NaN;
if useLead
    Ct = C;
    for kd = C.Kd:0.002:0.5
        Ct.Kd = kd;
        mg = loop_margins(M, Ct, ax.TsPM);
        if mg.PM >= ax.pmTarget, break, end
    end
    C.Kd = Ct.Kd; C.pmTarget = ax.pmTarget; C.TsPM = ax.TsPM;
end
if ax.bwHz > 1/(20*Ts)
    warning('%s: bandwidth %.1f Hz is high for Ts = %.2g s (rule of thumb fs > 20*bw).', ...
            M.name, ax.bwHz, Ts);
end
end
