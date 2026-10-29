%RUN_RIG  Real-time control of the eye-tracking rig. All control runs here in
%   MATLAB; the NUCLEO-H753ZI is an I/O bridge (firmware/, interface/protocol.md).
%
%   MODE = 'virtual' : simulated Nucleo + motor + camera, no hardware (start here)
%   MODE = 'serial'  : the real rig on P.link.port.  !! MOVES THE MOTOR AND SERVOS !!
%
%   TASK = 'io_check'        link, encoder, endstop and LED check; one short +2 V
%                            motor pulse after you press Enter (belt off first!)
%   TASK = 'sysid'           homing, then a 3 V open-loop step -> rotor inertia for
%                            params_local.m (run once after building the rig)
%   TASK = 'track_synthetic' homing, then follow a simulated moving head (no camera)
%   TASK = 'track_camera'    homing, then follow the user's head with the camera,
%                            aim the servo LEDs at the eyes, show the gaze point
%
%   Stop: close the live window or Ctrl-C. On exit the motor is stopped and the
%   LEDs switched off (onCleanup -> STOP frame); the firmware watchdog also stops
%   the motor 100 ms after MATLAB stops sending.

clear; close all;
root = setup_paths();

MODE = 'virtual';            % 'virtual' | 'serial'
TASK = 'track_camera';       % 'io_check' | 'sysid' | 'track_synthetic' | 'track_camera'
TRUN = 20;                   % run time after homing [s] (tracking tasks)
SHOW = true;                 % live window
% automated tests (matlab/tests/test_run_rig.m) override these via environment variables
if ~isempty(getenv('RIG_MODE')), MODE = getenv('RIG_MODE'); end
if ~isempty(getenv('RIG_TASK')), TASK = getenv('RIG_TASK'); end
if ~isempty(getenv('RIG_TRUN')), TRUN = str2double(getenv('RIG_TRUN')); end
if ~isempty(getenv('RIG_SHOW')), SHOW = strcmp(getenv('RIG_SHOW'), '1'); end

P = params();
assert(strcmp(P.loopMode, 'matlab'), 'set P.loopMode = ''matlab'' in params.m');
M = build_axis_model(P.ax(1), P);
C = design_pid(M, P.ax(1), P.Ts);
realtime = strcmp(MODE, 'serial');
s = P.srv; radPerUs = s.range/(s.pulseMax - s.pulseMin);
toUs = @(ang, j) min(max(s.zeroUs(j) + s.dir(j)*ang/radPerUs, s.pulseMin), s.pulseMax);
ledsOff = false(1, P.led.n);

io = rig_open(MODE, P);
cleanupRig = onCleanup(@() rig_close(io));
cmd = struct('duty', 0, 'servoUs', s.zeroUs, 'led', ledsOff);
tk = struct('t0', tic, 'rt', realtime, 'Ts', P.Ts);   % real-time pacing (see tick_wait)

%% ------------------------------------------------------------ io_check
if strcmp(TASK, 'io_check')
    fprintf('\n--- io_check: motor 0, LEDs blink, reading encoder + endstop for 3 s ---\n');
    tk.t0 = tic; c0 = NaN; nOk = 0;
    for k = 1:round(3/P.Ts)
        cmd.led = repmat(mod(floor(k*P.Ts*2), 2) == 1, 1, P.led.n);
        [io, fb] = rig_exchange(io, cmd);
        if fb.ok
            nOk = nOk + 1; if isnan(c0), c0 = double(fb.count); end
            if mod(k, round(0.5/P.Ts)) == 0
                fprintf('count %8d (%+6d)  endMin %d  endMax %d  rxErrors %d\n', fb.count, ...
                        double(fb.count) - c0, fb.endMin, fb.endMax, fb.rxErrors);
            end
        end
        rig_tick_wait(tk, k);
    end
    fprintf('replies %d of %d, link errors %d, timeouts %d\n', nOk, round(3/P.Ts), io.nErr, io.nTimeout);
    fprintf('-> turn the motor shaft by hand: ~700 counts/rev (x1) or ~2800 (x4); press the endstop: endMin = 1\n');
    if realtime
        input('Motor will get +2 V for 0.3 s. Belt OFF? Enter = go, Ctrl-C = abort: ', 's');
    end
    cmd.led = ledsOff; c1 = NaN; tk.t0 = tic;
    for k = 1:round(0.6/P.Ts)
        cmd.duty = (k*P.Ts < 0.3)*2/M.Vmax*P.drv.pwmCounts;
        [io, fb] = rig_exchange(io, cmd);
        if isnan(c1) && fb.ok, c1 = double(fb.count); end
        rig_tick_wait(tk, k);
    end
    cmd.duty = 0; [io, fb] = rig_exchange(io, cmd);
    dc = double(fb.count) - c1;
    fprintf('pulse moved %+d counts (%+.1f mm). Positive = toward MAX. If negative: flip MOTOR_DIR_POSITIVE_HIGH in config.h\n', ...
            dc, dc*M.q*M.r*1e3);
    return
end

%% ------------------------------------------------------------ homing
fprintf('Homing (%s)...\n', P.home.method);
cmd.duty = -P.home.duty*P.drv.pwmCounts;
homed = false; nStill = 0; lastCnt = NaN; tk.t0 = tic;
for k = 1:round(20/P.Ts)
    [io, fb] = rig_exchange(io, cmd);
    if fb.ok
        if strcmp(P.home.method, 'endstop')
            if fb.endMin, homed = true; break, end
        else
            if ~isnan(lastCnt) && abs(double(fb.count) - lastCnt) <= 1, nStill = nStill + 1; else, nStill = 0; end
            lastCnt = double(fb.count);
            if k*P.Ts > 0.3 && nStill >= round(P.home.tStall/P.Ts), homed = true; break, end
        end
    end
    rig_tick_wait(tk, k);
end
cmd.duty = 0; [io, fb] = rig_exchange(io, cmd);
if ~homed, error('run_rig:homing', 'Homing failed after 20 s (endstop not reached?)'); end
count0 = double(fb.count);
if strcmp(P.home.method, 'endstop'), xHome = P.belt.xMin; else, xHome = P.belt.xStop; end
th0 = xHome/M.r;
yOf = @(fb) th0 + (double(fb.count) - count0)*M.q;
fprintf('Homed at count %d (x = %.0f mm)\n', count0, xHome*1e3);

%% ------------------------------------------------------------ sysid
if strcmp(TASK, 'sysid')
    fprintf('Moving to the rail centre...\n');
    [io, y] = rig_move_to(io, 0.5*(P.belt.xMin + P.belt.xMax)/M.r, yOf, C, M, P, tk);
    cmd.duty = 0; tk.t0 = tic;
    for k = 1:round(0.5/P.Ts), [io, fb] = rig_exchange(io, cmd); rig_tick_wait(tk, k); end   % settle
    Vstep = 3; n = round(0.6/P.Ts);
    Ls = struct('t', (0:n-1)*P.Ts, 'y', nan(1, n));
    tk.t0 = tic;
    for k = 1:n
        cmd.duty = (k*P.Ts <= 0.5)*Vstep/M.Vmax*P.drv.pwmCounts;
        [io, fb] = rig_exchange(io, cmd);
        if fb.ok, Ls.y(k) = yOf(fb); end
        if fb.ok && fb.endMax, cmd.duty = 0; end
        rig_tick_wait(tk, k);
    end
    cmd.duty = 0; rig_exchange(io, cmd);
    idx = Ls.t < 0.5 & ~isnan(Ls.y);
    [a, k_, J, fit] = sysid_step(Ls.t(idx), Ls.y(idx), Vstep, P);
    d = fullfile(root, 'artifacts'); if ~exist(d, 'dir'), mkdir(d); end
    save(fullfile(d, sprintf('sysid_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))), 'Ls', 'P', 'fit');
    return
end

%% ------------------------------------------------------------ tracking
n = round(TRUN/P.Ts);
t = (0:n-1)*P.Ts;
useCam = strcmp(TASK, 'track_camera');
if strcmp(TASK, 'track_synthetic')
    Rref = coordinator_setpoints(P, t);          % simulated head path
end
if useCam
    if realtime, P.cam.source = 'webcam'; else, P.cam.source = 'synthetic'; end
    cam = camera_open(P);
    cleanupCam = onCleanup(@() camera_close(cam));
    T = eye_tracker_init(P);
    kf = [];
    gcal = gaze_calib_file(P, 'load');
    P.bSP = P.trk.bSP;                           % no set-point weighting while tracking
    tLastFrame = NaN; Gs = [];
end
if SHOW && useCam, V = viz_live_init(P, sprintf('run_rig %s (%s)', TASK, MODE)); end

y = yOf(fb);
st = struct('I', C.Kp*(1 - P.bSP)*y, 'D', 0, 'yPrev', y);
refPrev = y; tiltCmd = [0 0];
refNow = [y; 0; 0]; vFF = 0; vFFraw = 0;
L = struct('task', TASK, 'mode', MODE, 't', t, 'ref', nan(3, n), 'y', nan(1, n), 'V', nan(1, n), ...
           'loopMs', nan(1, n), 'ok', false(1, n), 'head', nan(3, n), 'statusCode', nan(1, n), ...
           'gaze', nan(2, n), 'frameMs', nan(1, n), 'nOverrun', 0);
codes = {'search', 'track', 'blink', 'lost'};
fprintf('Running %s for %.0f s (%s). Close the window or Ctrl-C to stop.\n', TASK, TRUN, MODE);
tk.t0 = tic; tPrevLoop = 0;
for k = 1:n
    tNow = toc(tk.t0);
    if ~realtime, tNow = t(k); end
    dt = tNow - tPrevLoop; tPrevLoop = tNow;
    [io, fb] = rig_exchange(io, cmd);
    if fb.ok, y = yOf(fb); end
    xCar = y*M.r;

    switch TASK
        case 'track_synthetic'
            refNow = Rref(:, k); vFF = 0;
        case 'track_camera'
            if mod(k - 1, P.cam.every) == 0
                tf0 = toc(tk.t0);
                ctx = struct('t', tNow, 'xCar', xCar, 'led', logical(P.led.on), 'tilt', tiltCmd);
                [cam, img, inf_] = camera_read(cam, P, ctx);
                if inf_.ok
                    tFrame = tNow; if realtime, tFrame = toc(tk.t0); end
                    det = eye_detect(img, P, eye_tracker_prior(T, tFrame, P));
                    [T, obs] = eye_tracker_update(T, det, tFrame, P);
                    [H, T] = head_estimate(obs, T, xCar, P);
                    dtf = tFrame - tLastFrame; if isnan(dtf), dtf = P.cam.every*P.Ts; end
                    tLastFrame = tFrame;
                    [kf, pHat, vHat] = eye_filter(kf, H.pos, dtf, P, H.valid);
                    if ~any(isnan(pHat)) && ~any(strcmp(obs.status, {'search', 'lost'}))
                        refNow = refs_from_eye(P, pHat);
                        inside = refNow(1)*M.r > P.belt.xMin && refNow(1)*M.r < P.belt.xMax;
                        vFFraw = P.trk.useVelFF*inside*vHat(1)/M.r;
                    else
                        refNow(1) = refPrev; vFFraw = 0;   % hold position while searching
                    end
                    G = [];
                    if ~isempty(gcal)
                        G = gaze_calib_apply(gcal, gaze_features(obs, H, P), P);
                        if G.valid
                            if isempty(Gs), Gs = G.uv; else, Gs = (1 - P.gcal.smooth)*G.uv + P.gcal.smooth*Gs; end
                            G.uv = Gs; G.px = Gs.*P.screen.res(:); L.gaze(:, k) = G.uv;
                        end
                    end
                    L.head(:, k) = H.pos; L.statusCode(k) = find(strcmp(codes, obs.status)) - 1;
                    L.frameMs(k) = (toc(tk.t0) - tf0)*1e3;
                    if SHOW
                        if ~viz_alive(V.fig), break, end
                        V = viz_live_update(V, tNow, img, det, obs, H, G, xCar, ...
                            struct('loop_ms', round(median_valid(L.loopMs(max(1,k-50):k))), ...
                                   'link_err', io.nErr, 'frame_ms', round(L.frameMs(k))));
                    end
                end
            end
    end
    if useCam                                    % low-passed velocity feed-forward
        aF = min(1, dt/P.trk.vffTau); vFF = vFF + aF*(vFFraw - vFF);
    end
    % carriage reference with slew limit, then PID (+ velocity feed-forward)
    dmax = P.ax(1).vmax*max(dt, P.Ts);
    ref = refNow(:);
    ref(1) = refPrev + min(max(ref(1) - refPrev, -dmax), dmax);
    ref(1) = min(max(ref(1), P.belt.xMin/M.r), P.belt.xMax/M.r);   % software limits
    refPrev = ref(1);
    Vff = M.Ke*vFF + M.R*M.Tc/M.Kt*tanh(vFF/0.05);
    [Vm, st] = pid_step(st, C, M, P, ref(1), y, Vff, dt);
    if ~fb.ok, Vm = 0; end                                       % no fresh encoder value: coast
    cmd.duty = Vm/M.Vmax*P.drv.pwmCounts;
    tiltCmd = ref(2:3)';
    cmd.servoUs = [toUs(ref(2), 1), toUs(ref(3), 2)];
    cmd.led = logical(P.led.on);

    L.ref(:, k) = ref; L.y(k) = y; L.V(k) = Vm; L.ok(k) = fb.ok;
    L.loopMs(k) = (toc(tk.t0) - tNow)*1e3*realtime;
    if ~rig_tick_wait(tk, k), L.nOverrun = L.nOverrun + 1; end
end
cmd.duty = 0; cmd.led = ledsOff; rig_exchange(io, cmd);

%% ------------------------------------------------------------ report
fprintf('Link errors: %d of %d, watchdog trips: %d, loop overruns: %d\n', io.nErr, n, io.nWdog, L.nOverrun);
if realtime
    fprintf('Loop time: median %.2f ms, max %.2f ms (budget %.0f ms)\n', median_valid(L.loopMs), max(L.loopMs), P.Ts*1e3);
end
d = fullfile(root, 'artifacts', 'logs'); if ~exist(d, 'dir'), mkdir(d); end
logFile = fullfile(d, sprintf('rig_%s_%s.mat', TASK, datestr(now, 'yyyymmdd_HHMMSS')));
save(logFile, 'L', 'P', 'C');
fprintf('Log: %s\n', logFile);
e = (L.ref(1,:) - L.y)*M.r*1e3;
fprintf('Carriage tracking error: RMS %.2f mm, max %.2f mm\n', sqrt(mean(e(~isnan(e)).^2)), max(abs(e)));
if useCam && ~realtime                          % virtual: compare with the virtual user's head
    ok = ~isnan(L.head(1,:));
    S = synth_scene(P, 'demo'); hx = arrayfun(@(tt) S.at(tt).head(1), t(ok));
    eh = (hx - L.y(ok)*M.r)*1e3;
    eh = eh(t(ok) > 2);
    fprintf('Camera offset from the head (true, t > 2 s): RMS %.1f mm, max %.1f mm\n', sqrt(mean(eh.^2)), max(abs(eh)));
end
if SHOW, plot_rig_log(L, P); end
