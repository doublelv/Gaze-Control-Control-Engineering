function mg = loop_margins(M, C, Ts)
%LOOP_MARGINS  Open-loop frequency response C(jw)G(jw)e^{-1.5 Ts jw} and margins.
%   The 1.5*Ts delay approximates ZOH (0.5 Ts) + one-sample compute delay.
%   No toolbox needed.

w = logspace(-1, log10(pi/Ts), 5000);
s = 1i*w;
G  = M.Kt ./ ( s .* ((M.L*s + M.R).*(M.J*s + M.b) + M.Kt*M.Ke) );
Cs = C.Kp + C.Ki./s + C.Kd*s./(C.Tf*s + 1);
Lw = Cs .* G .* exp(-s*1.5*Ts);

mag = abs(Lw);
ph  = unwrap(angle(Lw))*180/pi;
if ph(1) > 0, ph = ph - 360; end   % low-frequency phase of a type-2 loop is -180

mg.w = w; mg.mag = mag; mg.ph = ph;

% Gain crossover (last crossing of |L| = 1 downward)
ic = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1, 'last');
if isempty(ic)
    mg.wc = NaN; mg.PM = NaN;
else
    f = log(mag(ic))/(log(mag(ic)) - log(mag(ic+1)));
    mg.wc = exp(log(w(ic)) + f*(log(w(ic+1)) - log(w(ic))));
    mg.PM = 180 + (ph(ic) + f*(ph(ic+1) - ph(ic)));
end

% Phase crossover above the gain crossover (phase going below -180)
start = 1; if ~isempty(ic), start = ic; end
ip = find(ph(start:end-1) > -180 & ph(start+1:end) <= -180, 1, 'first');
if isempty(ip)
    mg.GM = Inf; mg.wpc = NaN;
else
    ip = ip + start - 1;
    mg.wpc = w(ip);
    mg.GM  = -20*log10(mag(ip));
end
end
