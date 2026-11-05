%RUN_CALIBRATION  9-point gaze calibration (PCCR + polynomial), saved locally.
%   Sit at the normal distance (40-65 cm), keep the head still and look at each
%   dot. The result goes to matlab/config/local/gaze_calibration.mat (git-ignored,
%   personal data). run_eye_demo and run_rig use it automatically.
%   SOURCE 'synthetic' runs the whole procedure with a virtual user (no hardware).
%   Hardware effects ('webcam' with LEDS = true): the Nucleo switches on the IR
%   LEDs in P.led.on for the whole calibration; the motor does not move and the
%   servos stay at P.srv.zeroUs. Use the SAME P.led.on as when tracking.

clear; close all;
root = setup_paths();
P = params();
SOURCE = 'synthetic';           % 'webcam' | 'synthetic'
SHOW   = true;                  % full-screen dots (false: headless, synthetic only)
LEDS   = true;                  % 'webcam' only: IR LEDs on via the Nucleo during calibration

P.cam.source = SOURCE;
if strcmp(SOURCE, 'synthetic')
    cam = camera_open(P, synth_scene(P, 'calib', [0; 0.2; 0]));
else
    cam = camera_open(P);
end
cleanupCam = onCleanup(@() camera_close(cam));
rig = [];
if LEDS && strcmp(SOURCE, 'webcam')
    rig.io = rig_open('serial', P);  cleanupRig = onCleanup(@() rig_close(rig.io));
    rig.cmd = struct('duty', 0, 'servoUs', P.srv.zeroUs, 'led', logical(P.led.on));
    fprintf('IR LEDs on: %s\n', mat2str(find(P.led.on)));
end
ui = struct('show', false);
if SHOW, ui = gaze_calib_window(P, 'open'); end
[V, X, info, rig] = gaze_calib_collect(P, cam, ui, 0, rig);
fprintf('Samples per point (right / left eye):\n'); disp(info.n);
bad = any(info.n < P.gcal.minSamples, 1);
if any(bad), fprintf('Points without enough samples: %s (eyes not detected?)\n', mat2str(find(bad))); end
M = gaze_calib_fit(V, X, P);
fprintf('Fit residual: %.1f / %.1f mm on the screen; leave-one-out error: %.1f / %.1f deg (right / left)\n', ...
        M.rmsM*1e3, M.loo);
if all(isnan(M.loo))
    error('Calibration failed: no eye had enough valid points. Check run_eye_demo first.');
end
f = gaze_calib_file(P, 'save', M);
fprintf('Saved: %s\n', f);
figure('Name', 'Calibration result');
plot(X(1,:)*1e3, X(2,:)*1e3, 'ko', 'MarkerSize', 12); hold on;
for e = 1:2
    if isempty(M.coef{e}), continue, end
    v = squeeze(V(:, e, :)); ok = all(~isnan(v), 1);
    pr = gaze_poly_terms(v(:, ok), M.orderE(e))*M.coef{e};
    plot(pr(:,1)*1e3, pr(:,2)*1e3, 'x', 'MarkerSize', 10, 'LineWidth', 1.5);
end
axis equal; grid on; xlabel('screen x [mm]'); ylabel('screen y [mm]');
legend('targets', 'right eye fit', 'left eye fit'); title('Calibration targets vs fitted gaze');
