function R = compare_log_sim(logFile, tWin, bSP, Cnew)
%COMPARE_LOG_SIM  Sim-to-real check: replay a real run_rig log through the model.
%   The logged carriage reference and the logged loop periods are fed to the
%   same PID + plant model (friction, saturation, PWM, encoder, one-loop
%   command delay) that make_paper_figures uses. The simulated carriage
%   position is compared with the measured one.
%     R = compare_log_sim('artifacts/logs/rig_track_camera_....mat')
%     R = compare_log_sim(file, [20 45])   % only this window of real time [s]
%   Real time is rebuilt from L.loopMs (L.t is the nominal k*Ts grid; every
%   loop overruns when the camera runs, so the nominal grid is too short).
if nargin < 2 || isempty(tWin), tWin = [-inf inf]; end
S = load(logFile); L = S.L; P = S.P; C = S.C;
if isfield(L, 'tReal'), tr = L.tReal; else
    lm = L.loopMs; lm(isnan(lm)) = 0; tr = cumsum(lm)/1e3;
end
k = find(~isnan(L.y) & L.loopMs > 0 & tr >= tWin(1) & tr <= tWin(2));
M = build_axis_model(P.ax(1), P);
P.bSP = P.trk.bSP;
if nargin >= 3 && ~isempty(bSP), P.bSP = bSP; end            % what-if: other set-point weight
if nargin >= 4 && ~isempty(Cnew), C = Cnew; end              % what-if: other PID gains
x = [L.y(k(1)); 0; 0]; Vapp = 0;
st = struct('I', 0, 'D', 0, 'yPrev', x(1)); holding = false;
n = numel(k); ySim = nan(1, n); VSim = nan(1, n);
dtSub = 1e-4; Mt = sub_model(P, dtSub);
for j = 1:n
    i = k(j);
    y = round(x(1)/M.q)*M.q; ySim(j) = x(1);
    if j < n, dt = tr(k(j+1)) - tr(i); else, dt = L.loopMs(i)/1e3; end
    ref = L.ref(1, i);
    steady = j > 1 && abs(ref - L.ref(1, k(j-1))) < 1e-9;
    [hold, holding] = hold_band(holding, steady, abs(ref - y)*M.r, P);   % (old logs: no hysteresis)
    if hold
        Vm = 0; st.yPrev = y; st.D = 0;
    else
        [Vm, st] = pid_step(st, C, M, P, ref, y, 0, dt);
    end
    if (y*M.r >= P.belt.xMax && Vm > 0) || (y*M.r <= P.belt.xMin && Vm < 0), Vm = 0; st.I = 0; end
    VSim(j) = Vm;
    for s = 1:max(1, round(dt/dtSub))                  % previous command acts during this loop
        tauF = Mt.Tc*tanh(x(2)/0.02);
        x = Mt.Ad*x + Mt.Bd*[Vapp; tauF];
    end
    Vapp = round(Vm/M.Vq)*M.Vq;
end
r = M.r;
R = struct('t', tr(k) - tr(k(1)), 'ref', L.ref(1,k)*r, 'yReal', L.y(k)*r, 'ySim', ySim*r, ...
           'VReal', L.V(k), 'VSim', VSim, 'head', L.head(1,k), 'loopMs', L.loopMs(k));
R.rmsDiff = sqrt(mean((R.yReal - R.ySim).^2));
R.rmsErrReal = sqrt(mean((R.ref - R.yReal).^2));
R.rmsErrSim  = sqrt(mean((R.ref - R.ySim).^2));
fprintf('%s: %d loops, %.1f s; real-sim RMS %.1f mm; tracking error RMS real %.1f / sim %.1f mm\n', ...
        logFile, n, R.t(end), R.rmsDiff*1e3, R.rmsErrReal*1e3, R.rmsErrSim*1e3);
end

function Mt = sub_model(P, dt)
P.Ts = dt; P.nSub = 1;
Mt = build_axis_model(P.ax(1), P);
end
