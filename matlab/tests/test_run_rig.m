function test_run_rig()
%TEST_RUN_RIG  End-to-end: the real run_rig script in virtual mode (simulated
%   Nucleo + motor + camera): io_check, homing + synthetic tracking, homing +
%   camera tracking, step and step_camera (zero steady-state error), gaze follow.
%   Checks link errors, tracking error and that the camera follows the virtual user's head.
envs = {'RIG_MODE', 'RIG_TASK', 'RIG_TRUN', 'RIG_SHOW', 'RIG_FACE_STUB', 'RIG_SOURCE', 'RIG_DET_METHOD', 'RIG_GZF'};
c = onCleanup(@() cellfun(@(e) setenv(e, ''), envs));
setenv('RIG_MODE', 'virtual'); setenv('RIG_SHOW', '0');
setenv('RIG_SOURCE', 'eyes');            % synthetic eyes (params.m default is 'face')
setenv('RIG_GZF', '0');                  % head following (gaze follow: own check at the end)
r = run_task('io_check', 0);
assert(r.io.nErr == 0, 'io_check link errors');
r = run_task('track_synthetic', 4);
assert(r.io.nErr == 0 && r.L.nOverrun == 0, 'link errors / overruns');
e = (r.L.ref(1,:) - r.L.y)*r.M.r*1e3;
% 1 Hz loop bandwidth (params.m, tuned on the rig) against the synthetic head
% (+-80 mm sine at 0.3 Hz + a 50 mm jump) gives ~45 mm RMS lag; tighten after
% the loop speed-up (faster loop -> higher bandwidth).
assert(sqrt(mean(e(r.L.t > 1).^2)) < 60, 'carriage tracking RMS %.1f mm', sqrt(mean(e(r.L.t > 1).^2)));
% step test (TASK = 'step'): first +50 mm step at t = 3 s, overshoot < 5 %, settled < 1.5 s
% (settled = within 5 % or within the hold band, where the motor is switched off on purpose)
r = run_task('step', 6);
k = find(r.L.tReal >= 3 & r.L.tReal < 6);
y0 = r.L.ref(1, k(1) - 1); ys = (r.L.y(k) - y0)*r.M.r; sz = r.P.stepTest.size;
sm = step_metrics(r.L.tReal(k) - 3, ys, 0, sz);
iout = find(abs(ys - sz) > max(0.05*sz, r.P.trk.holdBand + 0.5e-3), 1, 'last'); ts = r.L.tReal(k(min(iout + 1, end))) - 3;
assert(sm.OS < 5, 'step overshoot %.1f %%', sm.OS);
assert(ts < 1.5, 'step settling %.2f s', ts);
% zero steady-state error: hold band 0.3 mm (hold_band.m), the integrator removes the rest
ess = abs(r.L.ref(1, k(end)) - r.L.y(k(end)))*r.M.r;
assert(ess <= r.P.trk.holdExit, 'step steady-state error %.2f mm', ess*1e3);
h = r.L.hold(k); assert(sum(abs(diff(h))) <= 2, 'hold band hunting: %d on/off switches', sum(abs(diff(h))));
% step_camera: same square wave while the (synthetic) camera and detection run
r = run_task('step_camera', 6);
k = find(r.L.tReal >= 3 & r.L.tReal < 6);
y0 = r.L.ref(1, k(1) - 1); ys = (r.L.y(k) - y0)*r.M.r;
sm = step_metrics(r.L.tReal(k) - 3, ys, 0, sz);
assert(sm.OS < 5, 'step_camera overshoot %.1f %%', sm.OS);
assert(abs(ys(end) - sz) <= r.P.trk.holdExit, 'step_camera final error %.2f mm', abs(ys(end) - sz)*1e3);
assert(nnz(~isnan(r.L.head(1, :))) > 20, 'step_camera: camera/detection did not run');
% same 1 Hz limit as above: the camera lags the moving synthetic head (~50 mm RMS)
r = run_task('track_camera', 5);
ok = ~isnan(r.L.head(1,:)) & r.L.t > 2;
S = synth_scene(r.P, 'demo');
hx = arrayfun(@(tt) S.at(tt).head(1), r.L.t(ok));
off = (hx - r.L.y(ok)*r.M.r)*1e3;
assert(sqrt(mean(off.^2)) < 70, 'camera-to-head offset RMS %.1f mm', sqrt(mean(off.^2)));
% same with P.trk.source = 'face' (stand-in face detector: bright face blob)
setenv('RIG_FACE_STUB', 'face_stub_synthetic'); setenv('RIG_SOURCE', 'face');
r = run_task('track_camera', 5);
setenv('RIG_FACE_STUB', '');
ok = ~isnan(r.L.head(1,:)) & r.L.t > 2;
assert(nnz(ok) > 50, 'face mode: too few head estimates');
hx = arrayfun(@(tt) S.at(tt).head(1), r.L.t(ok));
off = (hx - r.L.y(ok)*r.M.r)*1e3;
assert(sqrt(mean(off.^2)) < 70, 'face mode: camera-to-head offset RMS %.1f mm', sqrt(mean(off.^2)));
% P.trk.source = 'both' (params.m default): eyes inside the face box, face as fallback
% (the synthetic face is a bright blob without a real eye band -> pupil detector;
%  eye_in_face is tested in test_eye_in_face)
setenv('RIG_FACE_STUB', 'face_stub_synthetic'); setenv('RIG_SOURCE', 'both'); setenv('RIG_DET_METHOD', 'pupil');
r = run_task('track_camera', 5);
setenv('RIG_FACE_STUB', ''); setenv('RIG_DET_METHOD', '');
ok = ~isnan(r.L.head(1,:)) & r.L.t > 2;
assert(nnz(ok) > 50, 'both: too few head estimates');
assert(any(r.L.srcCode == 2), 'both: the eyes were never followed');
hx = arrayfun(@(tt) S.at(tt).head(1), r.L.t(ok));
off = (hx - r.L.y(ok)*r.M.r)*1e3;
assert(sqrt(mean(off.^2)) < 70, 'both: camera-to-head offset RMS %.1f mm', sqrt(mean(off.^2)));
fprintf('both: followed eyes %d, face %d, none %d frames\n', nnz(r.L.srcCode == 2), nnz(r.L.srcCode == 1), nnz(r.L.srcCode == 0));
% gaze follow on (params.m default): runs, zeroes, and stays inside the software limits
setenv('RIG_FACE_STUB', 'face_stub_synthetic'); setenv('RIG_SOURCE', 'both'); setenv('RIG_DET_METHOD', 'pupil'); setenv('RIG_GZF', '1');
r = run_task('track_camera', 10);                % zeroing, then gaze follow
setenv('RIG_FACE_STUB', ''); setenv('RIG_DET_METHOD', ''); setenv('RIG_GZF', '0');
assert(isfield(r, 'gz') && any(strcmp(r.gz.phase, {'run', 'off'})), 'gaze follow: zeroing did not finish (%s)', r.gz.phase);
assert(any(~isnan(r.L.gazeM)), 'gaze follow: no raw angles logged');
x = r.L.y(~isnan(r.L.y))*r.M.r;
assert(all(x > r.P.belt.xMin - 0.005 & x < r.P.belt.xMax + 0.005), 'gaze follow: carriage outside the limits');
fprintf('gaze follow: phase %s, psi %+.1f deg, carriage %.0f..%.0f mm\n', r.gz.phase, r.gz.psi, min(x)*1e3, max(x)*1e3);
% the three demo modes: face only, gaze only (both = track_camera above)
setenv('RIG_FACE_STUB', 'face_stub_synthetic'); setenv('RIG_SOURCE', 'both'); setenv('RIG_DET_METHOD', 'pupil'); setenv('RIG_GZF', '');
r = run_task('track_face', 5);
assert(all(isnan(r.L.gazeDeg)), 'track_face: gaze follow should be off');
ok = ~isnan(r.L.head(1,:)) & r.L.t > 2; hx = arrayfun(@(tt) S.at(tt).head(1), r.L.t(ok));
off = (hx - r.L.y(ok)*r.M.r)*1e3;
assert(sqrt(mean(off.^2)) < 70, 'track_face: camera-to-head offset RMS %.1f mm', sqrt(mean(off.^2)));
r = run_task('track_gaze', 8);
setenv('RIG_FACE_STUB', ''); setenv('RIG_DET_METHOD', ''); setenv('RIG_GZF', '0');
assert(isfield(r, 'gz') && any(strcmp(r.gz.phase, {'run', 'off'})), 'track_gaze: zeroing did not finish');
x = r.L.y(~isnan(r.L.y))*r.M.r;
assert(all(x > r.P.belt.xMin - 0.005 & x < r.P.belt.xMax + 0.005), 'track_gaze: carriage outside the limits');
fprintf('track_face / track_gaze ok\n');
end

function r = run_task(task, trun)
setenv('RIG_TASK', task); setenv('RIG_TRUN', num2str(trun));
r = run_isolated();
end

function r = run_isolated()
% run the script in this function's workspace and return the interesting variables
run_rig;
r = struct('io', io);
if exist('L', 'var'), r.L = L; r.M = M; r.P = P; end
if exist('gz', 'var') && ~isempty(gz), r.gz = gz; end
end
