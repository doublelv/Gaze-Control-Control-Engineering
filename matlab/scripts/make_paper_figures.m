function make_paper_figures()
%MAKE_PAPER_FIGURES  Control-analysis figures for the paper / poster (no hardware).
%   All numbers come from params.m (the values tuned on the rig).
%   1. Plant: Bode of motor voltage -> carriage position, poles
%   2. PID design: open-loop Bode + margins, designed Ts (20 ms) vs measured loop (46 ms)
%   3. 50 mm carriage step (nonlinear model: friction, saturation, PWM, encoder)
%   4. Camera loop: 100 mm head step, camera latency 0.10 s,
%      WITHOUT vs WITH latency compensation (carriage position at frame time)
%   5. (optional) real logs: put rig_*.mat / sysid_*.mat in artifacts/ and see
%      plot_paper_logs.m
%   Output: artifacts/paper/*.png + paper_data.mat (git-ignored)
%   Runs in MATLAB (no toolboxes) and GNU Octave.

close all;
root = setup_paths();
P = params();
outDir = fullfile(root, 'artifacts', 'paper');
if ~exist(outDir, 'dir'), mkdir(outDir); end
TloopMeas = 0.046;                       % measured median loop period on the rig [s]

P.bSP = P.trk.bSP;                       % as in run_rig tracking
M = build_axis_model(P.ax(1), P);
C = design_pid(M, P.ax(1), P.Ts);
r = M.r;
D = struct();                            % everything plotted is also saved here

%% 1. Plant
w = logspace(-1, 4, 2000); s = 1i*w;
G = M.Kt ./ (s .* ((M.L*s + M.R).*(M.J*s + M.b) + M.Kt*M.Ke));    % V -> theta
Gx = G*r;                                                         % V -> x [m]
den = conv([1 0], conv([M.L M.R], [M.J M.b]) + [0 0 M.Kt*M.Ke]);  % s((Ls+R)(Js+b)+KtKe)
poles = roots(den);
D.plant = struct('w', w, 'magdB', 20*log10(abs(Gx)), 'phase', unwrap(angle(Gx))*180/pi, ...
                 'poles', poles, 'J', M.J, 'Kt', M.Kt, 'Ke', M.Ke, 'R', M.R, 'L', M.L, ...
                 'a', M.a, 'k', M.k, 'r', r);
fprintf('Plant V -> theta: J = %.3g kg m^2, a = %.2f 1/s, k = %.2f rad/(V s^2)\n', M.J, M.a, M.k);
fprintf('  poles: %s\n', mat2str(poles.', 4));

f1 = figure('Name', 'plant', 'Color', 'w');
subplot(2,1,1); semilogx(w, D.plant.magdB, 'LineWidth', 1.2); grid on;
ylabel('|G_x| [dB, m/V]'); title('Plant: motor voltage to carriage position');
subplot(2,1,2); semilogx(w, D.plant.phase, 'LineWidth', 1.2); grid on;
ylabel('phase [deg]'); xlabel('\omega [rad/s]');
save_fig(f1, outDir, 'fig_plant_bode');

%% 2. PID design: margins at the design Ts and at the measured loop period
mgD = loop_margins(M, C, P.Ts);
mgR = loop_margins(M, C, TloopMeas);
D.pid = struct('Kp', C.Kp, 'Ki', C.Ki, 'Kd', C.Kd, 'Tf', C.Tf, 'bwHz', P.ax(1).bwHz, ...
               'Ts', P.Ts, 'TloopMeas', TloopMeas, 'design', mgD, 'measured', mgR);
fprintf('PID (V/rad): Kp %.3f  Ki %.3f  Kd %.4f\n', C.Kp, C.Ki, C.Kd);
fprintf('  Ts %2.0f ms: PM %.1f deg, GM %.1f dB, wc %.2f Hz\n', P.Ts*1e3, mgD.PM, mgD.GM, mgD.wc/2/pi);
fprintf('  Ts %2.0f ms: PM %.1f deg, GM %.1f dB, wc %.2f Hz\n', TloopMeas*1e3, mgR.PM, mgR.GM, mgR.wc/2/pi);
% the same comparison for the original 2 Hz design and the first 1 Hz design (zeta 0.8)
ax0 = P.ax(1); ax0.zeta = 0.8; ax0.pmTarget = NaN; C0 = design_pid(M, ax0, P.Ts);   % first rig design (PI)
mg0D = loop_margins(M, C0, P.Ts); mg0R = loop_margins(M, C0, TloopMeas);
D.pid0 = struct('Kp', C0.Kp, 'Ki', C0.Ki, 'Kd', C0.Kd, 'design', mg0D, 'measured', mg0R);
fprintf('  (1 Hz, zeta 0.8: Kp %.2f, PM %.1f deg at %2.0f ms, %.1f deg at %2.0f ms)\n', C0.Kp, mg0D.PM, P.Ts*1e3, mg0R.PM, TloopMeas*1e3);
ax2 = P.ax(1); ax2.bwHz = 2; ax2.zeta = 0.8; ax2.pmTarget = NaN; C2 = design_pid(M, ax2, P.Ts);   % pole placement only
mg2D = loop_margins(M, C2, P.Ts); mg2R = loop_margins(M, C2, TloopMeas);
D.pid2 = struct('Kp', C2.Kp, 'Ki', C2.Ki, 'Kd', C2.Kd, 'design', mg2D, 'measured', mg2R);
fprintf('  (2 Hz design: PM %.1f deg at %2.0f ms, %.1f deg at %2.0f ms)\n', mg2D.PM, P.Ts*1e3, mg2R.PM, TloopMeas*1e3);

f2 = figure('Name', 'pid', 'Color', 'w');
subplot(2,1,1);
semilogx(mgD.w, 20*log10(mgD.mag), mgR.w, 20*log10(mgR.mag), '--', 'LineWidth', 1.2); grid on; hold on;
plot([mgD.w(1) mgD.w(end)], [0 0], 'k:');
ylabel('|C G| [dB]');
legend(sprintf('T_s = %.0f ms: PM %.0f deg, GM %.0f dB', P.Ts*1e3, mgD.PM, mgD.GM), ...
       sprintf('T = %.0f ms: PM %.0f deg, GM %.0f dB', TloopMeas*1e3, mgR.PM, mgR.GM), 'Location', 'southwest');
title(sprintf('Open loop, PID %g Hz (K_p %.2f, K_i %.2f, K_d %.0f)', P.ax(1).bwHz, C.Kp, C.Ki, C.Kd));
subplot(2,1,2);
semilogx(mgD.w, mgD.ph, mgR.w, mgR.ph, '--', 'LineWidth', 1.2); grid on; hold on;
plot([mgD.w(1) mgD.w(end)], [-180 -180], 'k:');
ylabel('phase [deg]'); xlabel('\omega [rad/s]'); ylim([-360 -90]);
save_fig(f2, outDir, 'fig_pid_margins');

%% 3. 50 mm step: first 1 Hz design (zeta 0.8, b = 1) vs final (params.m: zeta 1.5, b = 0.6)
xStep = 0.05;
Pold = P; Pold.bSP = 1;
[tA, yA] = sim_carriage_step(Pold, M, C0, xStep, TloopMeas, 4);
[tB, yB] = sim_carriage_step(P, M, C, xStep, TloopMeas, 4);
[tC, yC] = sim_carriage_step(P, M, C, xStep, P.Ts, 4);
smA = step_metrics(tA, yA, 0, xStep); smB = step_metrics(tB, yB, 0, xStep); smC = step_metrics(tC, yC, 0, xStep);
ts5 = @(t, y) t(min(find(abs(y - xStep) > 0.05*xStep, 1, 'last') + 1, numel(t)));
D.step = struct('tOld', tA, 'xOld', yA, 't', tB, 'x', yB, 't20', tC, 'x20', yC, ...
                'old', smA, 'new', smB, 'new20', smC, 'ts5Old', ts5(tA, yA), 'ts5', ts5(tB, yB), 'ts5_20', ts5(tC, yC));
fprintf('50 mm step @%2.0f ms  zeta 0.8, b = 1  : OS %4.1f %%, settle(5%%) %.2f s, ess %.1f mm\n', TloopMeas*1e3, smA.OS, D.step.ts5Old, smA.ess*1e3);
fprintf('50 mm step @%2.0f ms  zeta 1.5, b = %.1f: OS %4.1f %%, settle(5%%) %.2f s, ess %.1f mm\n', TloopMeas*1e3, P.bSP, smB.OS, D.step.ts5, smB.ess*1e3);
fprintf('50 mm step @%2.0f ms  zeta 1.5, b = %.1f: OS %4.1f %%, settle(5%%) %.2f s, ess %.1f mm\n', P.Ts*1e3, P.bSP, smC.OS, D.step.ts5_20, smC.ess*1e3);

f3 = figure('Name', 'step', 'Color', 'w');
plot(tA, yA*1e3, '--', tB, yB*1e3, tC, yC*1e3, '-.', 'LineWidth', 1.2); grid on; hold on;
plot([0 tA(end)], xStep*1e3*[1 1], 'k:');
xlabel('t [s]'); ylabel('carriage x [mm]');
legend(sprintf('first design (\\zeta 0.8, b 1), %.0f ms loop', TloopMeas*1e3), ...
       sprintf('final (\\zeta 1.5, b %.1f), %.0f ms loop', P.bSP, TloopMeas*1e3), ...
       sprintf('final, %.0f ms loop', P.Ts*1e3), 'Location', 'southeast');
title(sprintf('%.0f mm carriage step (model)', xStep*1e3));
save_fig(f3, outDir, 'fig_step');

%% 4. Camera loop with latency: without vs with compensation
head = @(t) 0.10*(t >= 0.5);               % the user moves 100 mm sideways at t = 0.5 s
T4 = 6;
Pn = P; Pn.trk.deadband = 0; Pn.trk.holdBand = 0;   % no dead/hold band: shows the latency effect alone
A = sim_camera_loop(Pn, M, C, head, TloopMeas, T4, false);
B = sim_camera_loop(Pn, M, C, head, TloopMeas, T4, true);
D.latency = struct('without', A, 'with', B, 'latency', P.cam.latency, 'Tloop', TloopMeas);
fprintf('Camera loop (latency %.0f ms, loop %.0f ms): without comp. last 2 s p-p %.0f mm, with %.0f mm\n', ...
        P.cam.latency*1e3, TloopMeas*1e3, ptp_tail(A, 2)*1e3, ptp_tail(B, 2)*1e3);

f4 = figure('Name', 'latency', 'Color', 'w');
plot(A.t, A.head*1e3, 'k:', A.t, A.x*1e3, B.t, B.x*1e3, 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('x [mm]');
legend('head', 'carriage, no compensation', 'carriage, compensated', 'Location', 'southeast');
title(sprintf('Camera latency %.0f ms, loop %.0f ms (no dead band)', P.cam.latency*1e3, TloopMeas*1e3));
save_fig(f4, outDir, 'fig_latency_comp');

save(fullfile(outDir, 'paper_data.mat'), 'D', 'P');
fprintf('Figures + paper_data.mat in %s\n', outDir);
end

%% ------------------------------------------------------------ helpers
function save_fig(f, d, name)
print(f, fullfile(d, [name '.png']), '-dpng', '-r200');
end

function v = ptp_tail(S, Ttail)
k = S.t > S.t(end) - Ttail; v = max(S.x(k)) - min(S.x(k));
end

function Mt = model_at(P, Tloop)
% plant discretised for a loop period Tloop (sub-steps of 0.1 ms)
P.Ts = Tloop; P.nSub = round(Tloop/1e-4);
Mt = build_axis_model(P.ax(1), P);
end

function [x, V] = plant_run(Mt, x, V, n)
for j = 1:n
    tauF = Mt.Tc*tanh(x(2)/0.02);                  % Coulomb friction (as in the virtual rig)
    x = Mt.Ad*x + Mt.Bd*[V; tauF];
end
end

function S = sim_camera_loop(P, M, C, head, Tloop, Tend, compensate)
% outer camera loop as in run_rig track_camera (face mode, no velocity feed-forward):
% each frame (every P.cam.every loops) shows the head RELATIVE to the camera as it
% was P.cam.latency ago. head estimate = carriage position + relative offset, where
% the carriage position is taken NOW (no compensation) or at the frame time (compensated).
Mt = model_at(P, Tloop); n = round(Tend/Tloop); t = (0:n-1)*Tloop;
x = zeros(3,1); Vapp = 0; refPrev = 0; refNow = 0;
st = struct('I', 0, 'D', 0, 'yPrev', 0); holding = false;
S = struct('t', t, 'x', zeros(1,n), 'ref', zeros(1,n), 'head', head(t));
lat = P.cam.latency;
for k = 1:n
    y = round(x(1)/M.q)*M.q; xCar = y*M.r; S.x(k) = x(1)*M.r;
    if mod(k - 1, P.cam.every) == 0 && t(k) > lat
        tf = t(k) - lat;                                   % frame exposure time
        xAtFrame = interp1(t(1:k), S.x(1:k), tf);          % carriage position then
        rel = head(tf) - xAtFrame;                         % what the image shows
        if compensate, xCam = xAtFrame; else, xCam = xCar; end
        hEst = xCam + rel;
        if abs(hEst - refNow*M.r) >= P.trk.deadband, refNow = hEst/M.r; end
    end
    dmax = P.ax(1).vmax*Tloop; ref = refPrev + min(max(refNow - refPrev, -dmax), dmax);
    ref = min(max(ref, P.belt.xMin/M.r), P.belt.xMax/M.r);
    steady = abs(ref - refPrev) < 1e-9; refPrev = ref; S.ref(k) = ref*M.r;
    [hold, holding] = hold_band(holding, steady, abs(ref - y)*M.r, P);
    if hold
        Vm = 0; st.yPrev = y; st.D = 0;
    else
        [Vm, st] = pid_step(st, C, M, P, ref, y, 0, Tloop);
    end
    [x, ~] = plant_run(Mt, x, Vapp, round(Tloop/Mt.dt));
    Vapp = round(Vm/M.Vq)*M.Vq;
end
end
