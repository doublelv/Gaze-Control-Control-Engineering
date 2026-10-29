%RUN_SIMULATION  Eye-tracking rig: full offline simulation with the ordered parts.
%   No hardware needed, no actuators move.
%   0. Hardware checks (current, torque, resolution, LED circuit)
%   1. Models: camera X = linear belt axis (FIT0186 + MDD10A), illuminators = SER0006
%   2. PID design for the DC axis + stability margins
%   3. Step test (staggered -> shows cross-axis coupling)
%   4. Tracking test with the OV9281 camera model: raw vs Kalman filter + feed-forward
%   5. Gaze reconstruction accuracy vs image noise (mono vs stereo)
%   All numbers come from params.m. Figures + summary go to ./results
%   Runs in plain MATLAB (no toolboxes) and in GNU Octave.

clear; close all; clc;
root = setup_paths();
P = params();
outDir = fullfile(root, 'artifacts', 'results');   % git-ignored
if ~exist(outDir, 'dir'), mkdir(outDir); end
fid = fopen(fullfile(outDir, 'summary.txt'), 'w');
log2 = @(varargin) cellfun(@(h) fprintf(h, varargin{:}), {1, fid});
r2d = 180/pi;

%% 0-2. Models, hardware checks, PID design, margins
nAx = numel(P.ax);
for i = 1:nAx
    M(i) = build_axis_model(P.ax(i), P); %#ok<SAGROW>
end
isDC = arrayfun(@(m) strcmp(m.type, 'dc'), M);
iDC = find(isDC); iSV = find(~isDC);
C = repmat(struct('Kp',NaN,'Ki',NaN,'Kd',NaN,'Tf',NaN,'wn',NaN), 1, nAx);
mg = struct('w',{},'mag',{},'ph',{},'wc',{},'PM',{},'wpc',{},'GM',{});
for i = iDC
    C(i)  = design_pid(M(i), P.ax(i), P.Ts);
    mg(i) = loop_margins(M(i), C(i), P.Ts);
end

% display units: linear axes in mm of carriage travel, rotary axes in deg
uScale = zeros(1,nAx); uName = cell(1,nAx);
for i = 1:nAx
    if M(i).r > 0, uScale(i) = M(i).r*1e3; uName{i} = 'mm';
    else,          uScale(i) = r2d;        uName{i} = 'deg';
    end
end

log2('=== 0. Hardware checks ===\n');
for i = iDC
    log2('%-18s DC : stall current %.1f A (MDD10A %.0f A/ch, PSU %.1f A) -> %s\n', M(i).name, ...
         M(i).Istall, P.drv.Imax, P.psu.Imax, ...
         pick(M(i).Istall < min(P.drv.Imax, P.psu.Imax), 'OK', 'TOO HIGH'));
    log2('%-18s     J_total %.2e kg m^2 (rotor share %.0f%%), gearbox eff. %.2f\n', '', ...
         M(i).J, 100*P.mot.N^2*P.mot.Jm/M(i).J, P.mot.eta);
    log2('%-18s     PWM %g kHz, %d counts -> %.2f mV/step; encoder %.3f deg/count\n', '', ...
         P.drv.fPWM/1e3, P.drv.pwmCounts, M(i).Vq*1e3, M(i).q*r2d);
    if M(i).r > 0
        log2('%-18s     belt: %.1f mm/rev, %.3f mm/count, max %.0f mm/s, max force %.0f N\n', '', ...
             2*pi*M(i).r*1e3, M(i).q*M(i).r*1e3, 251*2*pi/60*M(i).r*1e3, ...
             M(i).Kt*M(i).Vmax/M(i).R/M(i).r);
    end
end
for i = iSV
    log2('%-18s SERVO: gravity %.4f Nm vs stall %.3f Nm (x%.0f) -> %s; dead band %.2f deg\n', ...
         M(i).name, M(i).mgl, P.srv.tauStall, M(i).torqueMargin, ...
         pick(M(i).torqueMargin > 2, 'OK', 'TOO LITTLE MARGIN'), M(i).deadAng*r2d);
end
for Vs = [3.3 5 12]
    I  = (Vs - P.led.VfTyp)/P.led.Rser;
    log2('IR LED + 68R from %4.1f V: I = %3.0f mA, P_R = %3.0f mW -> %s\n', Vs, I*1e3, I^2*P.led.Rser*1e3, ...
         pick(I > 1.2*P.led.If, 'TOO MUCH CURRENT', pick(I*P.led.Rser*I > 0.5*P.led.Prated, 'OK (resistor warm)', 'OK')));
end
log2('NOTES: SER0006 needs 4.8-6 V; LEDs need a transistor/MOSFET (STM32 pin ~20 mA max).\n');

log2('\n=== 1. PID design for DC axes (pole placement) ===\n');
log2('%-18s %9s %9s %9s %9s %9s %8s %8s\n', 'Axis', 'J[kgm2]', 'Kp', 'Ki', 'Kd', 'wc[Hz]', 'PM[deg]', 'GM[dB]');
for i = iDC
    log2('%-18s %9.2e %9.2f %9.1f %9.4f %9.2f %8.1f %8.1f\n', M(i).name, M(i).J, ...
         C(i).Kp, C(i).Ki, C(i).Kd, mg(i).wc/2/pi, mg(i).PM, mg(i).GM);
end
log2('(gains in V/rad of OUTPUT-shaft angle; servo axes have no PID on our side)\n');

%% 3. Step test
t = 0:P.Ts:P.step.Tend;
th0 = [0; 10/r2d; 10/r2d];
stepOut = P.step.size(:);
if M(1).r > 0, stepOut(1) = P.step.size(1)/M(1).r; end
Rs = repmat(th0, 1, numel(t));
for i = 1:nAx
    Rs(i, t >= P.step.tStart(i)) = th0(i) + stepOut(i);
end
S = simulate_rig(P, M, C, t, Rs);

log2('\n=== 2. Step test (carriage %g mm, tilt %g deg) ===\n', P.step.size(1)*1e3, P.step.size(2)*r2d);
log2('%-18s %6s %8s %8s %8s %12s %16s\n', 'Axis', 'unit', 'tr[ms]', 'OS[%]', 'ts[ms]', 'ess[unit]', 'coupling[mdeg]');
for i = 1:nAx
    t0 = P.step.tStart(i);
    if i < nAx, t1 = P.step.tStart(i+1); else, t1 = P.step.Tend; end
    idx = t >= t0 & t < t1;
    sm = step_metrics(t(idx), S.th(i,idx), th0(i), th0(i) + stepOut(i));
    cpl = NaN;
    if i > 1
        idc = t >= P.step.tStart(1) & t < P.step.tStart(2);
        cpl = max(abs(S.th(i,idc) - th0(i)))*r2d*1e3;
    end
    log2('%-18s %6s %8.1f %8.1f %8.1f %12.4f %16.2f\n', M(i).name, uName{i}, sm.tr*1e3, sm.OS, ...
         sm.ts*1e3, sm.ess*uScale(i), cpl);
end

%% 4. Tracking test: does the eye stay centred and lit?
%  Three cases, all with the same moving head (incl. a sudden shift and 2 blinks):
%   A  ideal sensor   : true eye position, no delay, no noise (upper bound)
%   B  camera, raw    : OV9281 model (120 fps, latency, noise, blinks), last value held
%   C  camera, filter : same camera + Kalman filter + latency prediction + velocity FF
t = 0:P.Ts:P.traj.Tend; dt = P.Ts; n = numel(t);
pEye  = eye_trajectory(P, t);
pMeas = camera_measure(P, t, pEye);

% B: hold the last valid measurement
pRaw = pMeas; last = P.traj.p0;
for k = 1:n
    if any(isnan(pRaw(:,k))), pRaw(:,k) = last; else, last = pRaw(:,k); end
end
% C: Kalman filter
pFil = zeros(3,n); vFil = zeros(3,n); st = [];
for k = 1:n
    isNew = k == 1 || ~isequaln(pMeas(:,k), pMeas(:,k-1));
    [st, ph, vh] = eye_filter(st, pMeas(:,k), dt, P, isNew && ~any(isnan(pMeas(:,k))));
    if any(isnan(ph)), ph = P.traj.p0; vh = zeros(3,1); end
    pFil(:,k) = ph; vFil(:,k) = vh;
end

cases = {'A ideal sensor', 'B camera raw', 'C camera+filter+FF'};
pUse  = {pEye, pRaw, pFil};
res = struct('T',{},'R',{},'uPx',{},'ledErr',{});
for c = 1:3
    Rc = refs_from_eye(P, pUse{c}, dt);
    Rd = zeros(size(Rc));
    if c == 3 && P.trk.useVelFF
        inTravel = Rc(1,:)*M(1).r > P.belt.xMin & Rc(1,:)*M(1).r < P.belt.xMax;
        Rd(1,:) = vFil(1,:)/M(1).r .* inTravel;
    end
    Pc = P; if c == 3, Pc.bSP = P.trk.bSP; end
    Tc = simulate_rig(Pc, M, C, t, Rc, Rd);
    % performance w.r.t. the TRUE eye
    xCar = Tc.th(1,:)*M(1).r;
    dz   = pEye(3,:) - P.geo.cam(3);
    res(c).uPx = P.cam.f_px*(pEye(1,:) - (P.geo.cam(1) + xCar))./dz;
    res(c).vPx = P.cam.f_px*(pEye(2,:) - P.geo.cam(2))./dz;
    % ideal tilt for the ACTUAL carriage position
    offs = [P.geo.illumA_off P.geo.illumB_off]; led = zeros(2,n);
    for j = 1:2
        Lx = P.geo.cam(1) + xCar + offs(1,j); Ly = P.geo.cam(2) + offs(2,j);
        aim = atan2(pEye(2,:) - Ly, hypot(pEye(1,:) - Lx, pEye(3,:) - offs(3,j)));
        led(j,:) = (Tc.th(j+1,:) - aim)*r2d;
    end
    res(c).ledErr = led; res(c).T = Tc; res(c).R = Rc;
end
T = res(3).T; Rt = res(3).R; uPx = res(3).uPx; vPx = res(3).vPx;
err = (Rt - T.th).*uScale(:);

log2('\n=== 3. Tracking test: eye offset in the image and LED aim error vs the TRUE eye ===\n');
log2('Camera: f = %.0f px, %d fps, latency %.0f ms, noise %.1f px, distance error %.0f%%, 2 blinks\n', ...
     P.cam.f_px, P.cam.fps, P.cam.latency*1e3, P.cam.sigmaPx, 100*P.cam.zRelErr);
log2('%-20s %10s %10s %12s %12s %10s %10s\n', 'case', 'u RMS[px]', 'u max[px]', 'x RMS[mm]', 'LED RMS[deg]', 'LED max', 'in beam%');
for c = 1:3
    xe = res(c).uPx.*(pEye(3,:) - P.geo.cam(3))/P.cam.f_px*1e3;
    le = res(c).ledErr(:);
    log2('%-20s %10.1f %10.1f %12.2f %12.2f %10.2f %10.1f\n', cases{c}, sqrt(mean(res(c).uPx.^2)), ...
         max(abs(res(c).uPx)), sqrt(mean(xe.^2)), sqrt(mean(le.^2)), max(abs(le)), ...
         100*mean(abs(le) < P.led.halfAngleDeg));
end
log2('(eye stays in the image if |u| < %d px; LED beam half-angle %.1f deg)\n', P.cam.W/2, P.led.halfAngleDeg);
log2('Case C motor: peak voltage %.2f V, peak current %.2f A\n', max(abs(T.V(1,:))), max(abs(T.i(1,:))));

%% 5. Gaze accuracy
fprintf('\nRunning gaze Monte-Carlo (%d trials x %d noise levels)...\n', P.gaze.nTrials, numel(P.gaze.sigmaPx));
G = gaze_sim(P);
log2('\n=== 4. Gaze (optical axis) error [deg] vs pixel noise (f = %g px) ===\n', P.cam.f_px);
log2('(error scales ~ sigma/f. With the 1.65 mm lens the 3-D gaze DIRECTION is not reliable;\n the rig tracks the eye POSITION, which section 3 shows works.)\n');
log2('camera/light collinearity index = %.3f (0 = degenerate for mono)\n', G.collinearity);
log2('%8s %12s %12s %12s %12s %14s\n', 'sigma[px]', 'mono mean', 'mono p95', 'stereo mean', 'stereo p95', 'mono |dc|[mm]');
for a = 1:numel(G.sigma)
    em = sort(G.errMono(a,:)); es = sort(G.errStereo(a,:));
    k95 = ceil(0.95*numel(em));
    log2('%8.2f %12.3f %12.3f %12.3f %12.3f %14.2f\n', G.sigma(a), mean(em), em(k95), ...
         mean(es), es(k95), mean(G.depthErrMono(a,:))*1e3);
end
fclose(fid);

%% Figures
cols = [0 0.447 0.741; 0.85 0.325 0.098; 0.466 0.674 0.188];

f1 = figure('Name', 'Step test', 'Position', [50 50 900 700]);
for i = 1:3
    subplot(4,1,i);
    plot(S.t, S.ref(i,:)*uScale(i), 'k--', S.t, S.th(i,:)*uScale(i), 'Color', cols(i,:), 'LineWidth', 1.2);
    ylabel(['[' uName{i} ']']); title(M(i).name); grid on; xlim([0 P.step.Tend]);
    if i == 1, legend('reference', 'actual', 'Location', 'southeast'); end
end
subplot(4,1,4);
plot(S.t, S.V(iDC,:)', 'LineWidth', 1.1); grid on;
ylabel('V [V]'); xlabel('t [s]'); title('FIT0186 motor voltage (MDD10A output)'); xlim([0 P.step.Tend]);
print(f1, fullfile(outDir, 'fig1_step.png'), '-dpng', '-r130');

f2 = figure('Name', 'Open-loop Bode', 'Position', [80 80 900 650]);
subplot(2,1,1);
for i = iDC, semilogx(mg(i).w/2/pi, 20*log10(mg(i).mag), 'Color', cols(i,:), 'LineWidth', 1.2); hold on; end
plot(xlim, [0 0], 'k:'); grid on; ylabel('|L| [dB]');
title(sprintf('Open loop C(s)G(s)e^{-1.5T_s s}: PM %.0f deg, GM %.1f dB', mg(iDC(1)).PM, mg(iDC(1)).GM));
subplot(2,1,2);
for i = iDC, semilogx(mg(i).w/2/pi, mg(i).ph, 'Color', cols(i,:), 'LineWidth', 1.2); hold on; end
plot(xlim, [-180 -180], 'k:'); grid on; ylabel('phase [deg]'); xlabel('f [Hz]');
print(f2, fullfile(outDir, 'fig2_bode.png'), '-dpng', '-r130');

f3 = figure('Name', 'Tracking', 'Position', [110 110 900 800]);
subplot(3,1,1);
plot(t, pEye(1,:)*1e3, 'k', t, res(2).T.th(1,:)*M(1).r*1e3, t, T.th(1,:)*M(1).r*1e3, 'LineWidth', 1.1);
grid on; ylabel('x [mm]'); legend('true eye', 'carriage B (raw)', 'carriage C (filter+FF)', 'Location', 'northwest');
title('Camera carriage following the eye');
subplot(3,1,2);
plot(t, res(1).uPx, t, res(2).uPx, t, res(3).uPx, 'LineWidth', 1.1); grid on;
ylabel('u [px]'); legend(cases, 'Location', 'northwest');
title(sprintf('Eye offset from the image centre (sensor edge at +-%d px)', P.cam.W/2));
subplot(3,1,3);
plot(t, res(2).ledErr(1,:), t, res(3).ledErr(1,:), 'LineWidth', 1.1); hold on;
plot(t([1 end]), [1 1]*P.led.halfAngleDeg, 'k:', t([1 end]), -[1 1]*P.led.halfAngleDeg, 'k:');
grid on; ylabel('[deg]'); xlabel('t [s]'); legend('LED A, B raw', 'LED A, C filter', 'Location', 'northwest');
title('LED aim error vs the true eye (dotted = beam half-angle)');
print(f3, fullfile(outDir, 'fig3_tracking.png'), '-dpng', '-r130');

f4 = figure('Name', 'Gaze accuracy', 'Position', [140 140 800 450]);
x = G.sigma;
plot(x, mean(G.errMono, 2), 'o-', 'Color', cols(1,:), 'LineWidth', 1.5); hold on;
plot(x, mean(G.errStereo, 2), 's-', 'Color', cols(2,:), 'LineWidth', 1.5);
pm = zeros(size(x)); ps = pm;
for a = 1:numel(x)
    em = sort(G.errMono(a,:)); es = sort(G.errStereo(a,:)); k95 = ceil(0.95*numel(em));
    pm(a) = em(k95); ps(a) = es(k95);
end
plot(x, pm, 'o--', 'Color', cols(1,:)); plot(x, ps, 's--', 'Color', cols(2,:));
grid on; xlabel('image noise \sigma [px]'); ylabel('optical-axis error [deg]');
legend('mono mean', 'stereo mean', 'mono p95', 'stereo p95', 'Location', 'northwest');
title(sprintf('Gaze reconstruction error vs image noise (f = %g px)', P.cam.f_px));
ylim([0 min(60, 1.1*max([pm ps]))]);
print(f4, fullfile(outDir, 'fig4_gaze.png'), '-dpng', '-r130');

fprintf('\nDone. Figures and summary.txt saved in %s\n', outDir);
