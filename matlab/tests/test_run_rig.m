function test_run_rig()
%TEST_RUN_RIG  End-to-end: the real run_rig script in virtual mode (simulated
%   Nucleo + motor + camera): io_check, homing + synthetic tracking, homing +
%   camera tracking. Checks link errors, tracking error and that the camera
%   follows the virtual user's head.
envs = {'RIG_MODE', 'RIG_TASK', 'RIG_TRUN', 'RIG_SHOW'};
c = onCleanup(@() cellfun(@(e) setenv(e, ''), envs));
setenv('RIG_MODE', 'virtual'); setenv('RIG_SHOW', '0');
r = run_task('io_check', 0);
assert(r.io.nErr == 0, 'io_check link errors');
r = run_task('track_synthetic', 4);
assert(r.io.nErr == 0 && r.L.nOverrun == 0, 'link errors / overruns');
e = (r.L.ref(1,:) - r.L.y)*r.M.r*1e3;
assert(sqrt(mean(e(r.L.t > 1).^2)) < 20, 'carriage tracking RMS %.1f mm', sqrt(mean(e(r.L.t > 1).^2)));
r = run_task('track_camera', 5);
ok = ~isnan(r.L.head(1,:)) & r.L.t > 2;
S = synth_scene(r.P, 'demo');
hx = arrayfun(@(tt) S.at(tt).head(1), r.L.t(ok));
off = (hx - r.L.y(ok)*r.M.r)*1e3;
assert(sqrt(mean(off.^2)) < 20, 'camera-to-head offset RMS %.1f mm', sqrt(mean(off.^2)));
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
end
