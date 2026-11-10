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
%   TASK = 'step'            homing, then carriage steps of P.stepTest.size around the
%                            rail centre (square wave, P.stepTest.period) -> step-response
%                            log for the sim-to-real comparison (no camera)
%   TASK = 'step_camera'     the same steps, but the camera, face and eye detection
%                            run as in track_camera (they only load the loop, the
%                            carriage follows the square wave) -> step response with
%                            the real loop time
%   TASK = 'track_camera'    homing, then follow the user's head with the camera
%                            (P.trk.source: 'both' = eyes inside the face, else the
%                            face), aim the servo LEDs at the eyes, all LEDs in
%                            P.led.on switched on, gaze point if calibrated;
%                            P.gzf.on: the carriage goes where the eyes look
%                            (look straight at the camera for P.gzf.tZero s first)
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
if ~isempty(getenv('RIG_SOURCE')), P.trk.source = getenv('RIG_SOURCE'); end   % tests: 'eyes' | 'face' | 'both'
if ~isempty(getenv('RIG_FACE_STUB'))             % tests: face mode with a stand-in detector
    if strcmp(P.trk.source, 'eyes'), P.trk.source = 'face'; end
    P.face.detectFcn = str2func(getenv('RIG_FACE_STUB'));
end
if ~isempty(getenv('RIG_DET_METHOD')), P.det.method = getenv('RIG_DET_METHOD'); end   % tests: 'pupil' | 'face'
if ~isempty(getenv('RIG_GZF')), P.gzf.on = strcmp(getenv('RIG_GZF'), '1'); end   % tests: gaze follow off/on
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

%% ------------------------------------------------------------ pre-check
% Short push toward MAX: the encoder must count UP. Catches a dead/unplugged
% encoder, missing 12 V, or swapped motor/encoder wires BEFORE anything moves far
% (otherwise homing "stalls" at once and the PID runs away).
fprintf('Pre-check: short move toward MAX...\n');
[io, fb] = rig_exchange(io, cmd); c0 = double(fb.count); tk.t0 = tic;
cmd.duty = P.home.duty*P.drv.pwmCounts;
for k = 1:round(P.home.tCheck/P.Ts), [io, fb] = rig_exchange(io, cmd); rig_tick_wait(tk, k); end
cmd.duty = 0; [io, fb] = rig_exchange(io, cmd);
for k = 1:round(0.2/P.Ts), [io, fb] = rig_exchange(io, cmd); rig_tick_wait(tk, k); end   % let it stop
dcChk = double(fb.count) - c0;
fprintf('  moved %+d counts (%+.1f mm)\n', round(dcChk), dcChk*M.q*M.r*1e3);
if dcChk < -P.home.minCheckCounts
    error('run_rig:direction', ['Motor drove toward MAX but the encoder counted DOWN (%+d). ', ...
          'Motor or encoder wires are swapped. Check with TASK = ''io_check''.'], round(dcChk));
elseif dcChk < P.home.minCheckCounts
    error('run_rig:noMotion', ['Encoder changed only %+d counts during the pre-check move. ', ...
          'Is the 12 V supply on? Is the encoder cable (and its power) plugged in? ', ...
          'Is the carriage already at the MAX (left) end? Check with TASK = ''io_check''.'], round(dcChk));
end

%% ------------------------------------------------------------ homing
fprintf('Homing (%s)...\n', P.home.method);
if strcmp(P.home.method, 'endstop')
    cmd.duty = -P.home.duty*P.drv.pwmCounts;
    homed = false; tk.t0 = tic;
    for k = 1:round(20/P.Ts)
        [io, fb] = rig_exchange(io, cmd);
        if fb.ok && fb.endMin, homed = true; break, end
        rig_tick_wait(tk, k);
    end
    cmd.duty = 0; [io, fb] = rig_exchange(io, cmd);
else
    [io, fb, homed] = rig_home_stall(io, P, -1, tk);          % MIN end (right)
end
if ~homed, error('run_rig:homing', 'Homing failed after 20 s (endstop not reached?)'); end
count0 = double(fb.count);
if strcmp(P.home.method, 'stall') && P.home.bothEnds
    % second stop: measure the real travel, recentre the limits on it
    fprintf('Homed MIN at count %d. Homing the MAX end (measuring the travel)...\n', count0);
    [io, fbMax, ok2] = rig_home_stall(io, P, +1, tk);
    if ~ok2, error('run_rig:homing', 'MAX end not reached after 20 s'); end
    travelM = (double(fbMax.count) - count0)*M.q*M.r;
    fprintf('Measured travel %.0f mm (expected %.0f mm)\n', travelM*1e3, P.belt.travel*1e3);
    if abs(travelM - P.belt.travel) > P.home.travelTol
        error('run_rig:travel', ['Measured travel %.0f mm differs from P.belt.travel = %.0f mm by more than %.0f mm: ', ...
              'belt slipping, encoder losing counts, or something blocking the carriage? ', ...
              '(If the rail really changed, set P.belt.travel.)'], travelM*1e3, P.belt.travel*1e3, P.home.travelTol*1e3);
    end
    fr = logical(P.led.onFrame) & P.led.pos(1,:) > P.belt.travel/2;   % frame LEDs at the MAX end
    P.led.pos(1, fr) = P.led.pos(1, fr) + (travelM - P.belt.travel);
    P.belt.travel = travelM;
    P.belt.xStop = -travelM/2;
    P.belt.xMin = P.belt.xStop + P.belt.margin; P.belt.xMax = -P.belt.xStop - P.belt.margin;
end
if strcmp(P.home.method, 'endstop'), xHome = P.belt.xMin; else, xHome = P.belt.xStop; end
th0 = xHome/M.r;
yOf = @(fb) th0 + (double(fb.count) - count0)*M.q;
fprintf('Homed at count %d (x = %.0f mm)\n', count0, xHome*1e3);
% start every task from the middle of the rail
fprintf('Moving to the rail centre...\n');
[io, y] = rig_move_to(io, 0.5*(P.belt.xMin + P.belt.xMax)/M.r, yOf, C, M, P, tk);

%% ------------------------------------------------------------ sysid
if strcmp(TASK, 'sysid')
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
useCam = any(strcmp(TASK, {'track_camera', 'step_camera'}));
followCam = strcmp(TASK, 'track_camera');       % step_camera: camera only loads the loop
if strcmp(TASK, 'track_synthetic')
    Rref = coordinator_setpoints(P, t);          % simulated head path
end
P.bSP = P.trk.bSP;                               % set-point weight of the tracking controller
useHold = useCam || strcmp(TASK, 'step');        % hold band: motor off near a still target
holding = false;                                 % hold_band state (hysteresis)
if useCam
    if realtime, P.cam.source = 'webcam'; else, P.cam.source = 'synthetic'; end
    cam = camera_open(P);
    cleanupCam = onCleanup(@() camera_close(cam));
    T = eye_tracker_init(P);
    kf = [];
    gcal = gaze_calib_file(P, 'load');
    tLastFrame = NaN; Gs = [];
    useFace = any(strcmp(P.trk.source, {'face', 'both'}));   % face box (no LEDs needed)
    useBoth = strcmp(P.trk.source, 'both');      % eyes when found inside the face, else the face
    fd = []; fbox = []; zFace = NaN; hFace = [];
    tHist = nan(1, n); xHist = nan(1, n);         % carriage history for the camera-latency compensation
    camLat = realtime*P.cam.latency;              % the virtual camera has no latency
    fprintf('Following: %s (P.trk.source)\n', P.trk.source);
    trkSrc = 'none';
    gz = [];                                     % gaze follow state (P.gzf, gaze_follow.m)
    if followCam && P.gzf.on
        fprintf(['Gaze follow ON: look AT THE CAMERA for the first ~2 s (zeroing).\n', ...
                 '  Then look left or right: the camera moves that way and stops when you look at it.\n']);
    end
end
if SHOW && useCam, V = viz_live_init(P, sprintf('run_rig %s (%s)', TASK, MODE)); end

cmd.duty = 0; [io, fb] = rig_exchange(io, cmd); if fb.ok, y = yOf(fb); end   % fresh position (after the centring move)
st = struct('I', C.Kp*(1 - P.bSP)*y, 'D', 0, 'yPrev', y);
refPrev = y; tiltCmd = [0 0];
refNow = [y; 0; 0]; vFF = 0; vFFraw = 0; yStep0 = y;
L = struct('task', TASK, 'mode', MODE, 't', t, 'ref', nan(3, n), 'y', nan(1, n), 'V', nan(1, n), ...
           'loopMs', nan(1, n), 'ok', false(1, n), 'head', nan(3, n), 'statusCode', nan(1, n), ...
           'gaze', nan(2, n), 'frameMs', nan(1, n), 'srcCode', nan(1, n), 'nOverrun', 0, 'nGuard', 0, 'tReal', nan(1, n), ...
           'gazeDeg', nan(1, n), 'glints', nan(1, n), 'hold', false(1, n), 'uPID', nan(3, n), ...
           'gazeM', nan(1, n), 'gazePhi', nan(1, n));
codes = {'search', 'track', 'blink', 'lost'};
fprintf('Running %s for %.0f s (%s). Close the window or Ctrl-C to stop.\n', TASK, TRUN, MODE);
Mlim = M; if useCam, Mlim.Vmax = min(M.Vmax, P.trk.Vlim); end   % voltage limit with the camera (pid_step saturates + anti-wind-up)
tk.t0 = tic; tPrevLoop = 0; tBigErr = 0; stopMsg = ''; aimF = [NaN NaN]; aimS = [NaN NaN];
for k = 1:n
    tNow = toc(tk.t0);
    if ~realtime, tNow = t(k); end
    dt = tNow - tPrevLoop; tPrevLoop = tNow;
    try
        [io, fb] = rig_exchange(io, cmd);
    catch err                                    % e.g. encoder jump (Nucleo reset): stop, keep the log
        stopMsg = err.message; break
    end
    if fb.ok, y = yOf(fb); end
    xCar = y*M.r;
    if useCam, tHist(k) = tNow; xHist(k) = xCar; end

    switch TASK
        case 'track_synthetic'
            refNow = Rref(:, k); vFF = 0;
        case {'step', 'step_camera'}             % square wave: centre, +size, centre, ...
            up = mod(floor(tNow/P.stepTest.period), 2) == 1;
            refNow(1) = yStep0 + up*P.stepTest.size/M.r; vFF = 0;   % (2:3): servos, set by the camera in step_camera
            if tNow > TRUN, break, end           % real time (n is only an upper bound)
    end
    if useCam
        if mod(k - 1, P.cam.every) == 0
            tf0 = toc(tk.t0);
            ctx = struct('t', tNow, 'xCar', xCar, 'led', logical(P.led.on), 'tilt', tiltCmd);
            [cam, img, inf_] = camera_read(cam, P, ctx);
            if inf_.ok
                tFrame = tNow; if realtime, tFrame = toc(tk.t0); end
                % carriage position WHEN THE FRAME WAS TAKEN (not now): otherwise the
                % carriage's own motion appears as head motion -> oscillation
                xCam = xCar;
                if camLat > 0
                    ih = max(1, k - 100):k; ih = ih(~isnan(tHist(ih)));
                    tc = min(max(tFrame - camLat, tHist(ih(1))), tHist(ih(end)));
                    if numel(ih) > 1, xCam = interp1(tHist(ih), xHist(ih), tc); end
                end
                imgE = img;
                if useFace                   % face first; pupils only inside the face
                    [fbox, fd] = face_find(fd, img, P, fbox);
                    imgE = face_mask(img, fbox);
                end
                if useFace && strcmp(P.det.method, 'face')
                    det = eye_in_face(img, fbox, P);   % iris inside the face box
                else
                    det = eye_detect(imgE, P, eye_tracker_prior(T, tFrame, P));
                end
                [T, obs] = eye_tracker_update(T, det, tFrame, P);
                [H, T] = head_estimate(obs, T, xCam, P);
                eyesTrk = ~any(strcmp(obs.status, {'search', 'lost'}));
                if useFace, [Hf, zFace] = face_head(fbox, H, zFace, xCam, P); end
                if useBoth && eyesTrk && H.valid
                    trkSrc = 'eyes';         % both: pupils found inside the face box
                elseif useFace
                    H = Hf; trkSrc = 'face';
                    if ~H.valid, trkSrc = 'none'; end
                else
                    trkSrc = 'eyes'; if ~eyesTrk, trkSrc = 'none'; end
                end
                L.srcCode(k) = find(strcmp({'none', 'face', 'eyes'}, trkSrc)) - 1;
                dtf = tFrame - tLastFrame; if isnan(dtf), dtf = P.cam.every*P.Ts; end
                tLastFrame = tFrame;
                [kf, pHat, vHat] = eye_filter(kf, H.pos, dtf, P, H.valid);
                trackOk = ~strcmp(trkSrc, 'none');
                % gaze follow: iris position inside the face box -> where the eyes look
                gzDeg = NaN;
                if followCam
                    [gz, gzDeg] = gaze_follow(gz, obs, fbox, H, xCam, tFrame, strcmp(trkSrc, 'eyes'), P);
                    L.gazeDeg(k) = gzDeg; L.gazeM(k) = gz.m; L.gazePhi(k) = gz.phi;
                end
                L.glints(k) = sum(arrayfun(@(x) size(x.glints, 1) > 0, det.eyes));
                if ~followCam && ~any(isnan(pHat)) && trackOk
                    refNew = refs_from_eye(P, pHat);
                    refNow(2:3) = refNew(2:3);   % step_camera: the servos still aim at the eyes
                elseif ~followCam
                    % keep the step reference
                elseif ~any(isnan(pHat)) && trackOk
                    refNew = refs_from_eye(P, pHat);
                    if P.gzf.on && ~isnan(gzDeg)   % camera toward where the eyes look
                        xg = pHat(1) + H.z*tand(gzDeg);
                        refNew(1) = min(max(xg, P.belt.xMin), P.belt.xMax)/M.r;
                    end
                    if abs(refNew(1)*M.r - refNow(1)*M.r) < P.trk.deadband
                        refNew(1) = refNow(1);   % dead band: ignore small head moves / jitter
                    end
                    refNow = refNew;
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
                gzLabel = ''; if ~isempty(gz), gzLabel = gz.label; end
                if SHOW
                    if ~viz_alive(V.fig), break, end
                    try
                    V = viz_live_update(V, tNow, img, det, obs, H, G, xCar, ...
                        struct('loop_ms', round(median_valid(L.loopMs(max(1,k-50):k))), ...
                               'link_err', io.nErr, 'frame_ms', round(L.frameMs(k)), 'follows', ...
                               [trkSrc gzLabel]));
                    if useFace && isfield(V, 'hDet') && ~isempty(V.hDet)   % magenta face box
                        if isempty(hFace) || ~ishghandle(hFace), hFace = plot(V.axCam, NaN, NaN, 'm-', 'LineWidth', 2); end
                        if isempty(fbox), set(hFace, 'XData', NaN, 'YData', NaN);
                        else, b = fbox; set(hFace, 'XData', b(1) + [0 b(3) b(3) 0 0] - 0.5, ...
                                                  'YData', b(2) + [0 0 b(4) b(4) 0] - 0.5); end
                    end
                    catch                        % window closed while drawing: stop, keep the log
                        break
                    end
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
    refSteady = abs(ref(1) - refPrev) < 1e-9;
    refPrev = ref(1);
    Vff = M.Ke*vFF + M.R*M.Tc/M.Kt*tanh(vFF/0.05);
    [hold, holding] = hold_band(holding, refSteady, abs(ref(1) - y)*M.r, P);
    if useHold && hold
        Vm = 0; st.yPrev = y; st.D = 0;          % at the target and target still: motor off, no hunting
    else
        [Vm, st] = pid_step(st, C, Mlim, P, ref(1), y, Vff, dt);
        L.uPID(:, k) = [st.uP; st.uI; st.uD];    % P, I, D terms [V] (plot_pid_terms)
    end
    if ~fb.ok, Vm = 0; end                                       % no fresh encoder value: coast
    % runaway check: the carriage cannot follow its (slew-limited) reference
    if fb.ok && abs(ref(1) - y)*M.r > P.trk.maxErr, tBigErr = tBigErr + max(dt, P.Ts); else, tBigErr = 0; end
    if tBigErr > 1.0
        cmd.duty = 0; rig_exchange(io, cmd);
        stopMsg = sprintf('Carriage %.0f mm away from its reference for 1 s -> STOPPED (encoder? wiring? blocked?)', ...
                          abs(ref(1) - y)*M.r*1e3);
        break                                    % log is saved below, then the error is raised
    end
    % encoder guard: past a software limit, never drive further outward (overshoot, stalls)
    if (xCar >= P.belt.xMax && Vm > 0) || (xCar <= P.belt.xMin && Vm < 0)
        Vm = 0; st.I = 0;                        % and drop the wound-up integrator
        L.nGuard = L.nGuard + 1;
    end
    cmd.duty = Vm/M.Vmax*P.drv.pwmCounts;
    % servo LEDs: low-pass + dead band on the aim angle (estimate jumps -> no twitching)
    aimRaw = ref(2:3)';
    if any(isnan(aimS)), aimS = aimRaw; aimF = aimRaw; end
    aimS = aimS + min(1, max(dt, P.Ts)/P.srv.tauAim)*(aimRaw - aimS);   % smoothed
    upd = abs(aimS - aimF)*180/pi >= P.srv.deadAim;                      % command moves in steps
    aimF(upd) = aimS(upd);
    if useCam, ref(2:3) = aimF'; end
    tiltCmd = ref(2:3)';
    cmd.servoUs = [toUs(ref(2), 1), toUs(ref(3), 2)];
    cmd.led = logical(P.led.on);

    L.ref(:, k) = ref; L.y(k) = y; L.V(k) = Vm; L.ok(k) = fb.ok; L.tReal(k) = tNow; L.hold(k) = useHold && hold;
    L.loopMs(k) = (toc(tk.t0) - tNow)*1e3*realtime;
    if ~rig_tick_wait(tk, k), L.nOverrun = L.nOverrun + 1; end
end
cmd.duty = 0; cmd.led = ledsOff; try, rig_exchange(io, cmd); catch, end   % (rig_close stops it too)

%% ------------------------------------------------------------ report
fprintf('Link errors: %d of %d, watchdog trips: %d, loop overruns: %d\n', io.nErr, n, io.nWdog, L.nOverrun);
if L.nGuard > 0, fprintf('Encoder guard stopped an outward move %d times (carriage past a software limit)\n', L.nGuard); end
if realtime
    fprintf('Loop time: median %.2f ms, max %.2f ms (budget %.0f ms)\n', median_valid(L.loopMs), max(L.loopMs), P.Ts*1e3);
end
d = fullfile(root, 'artifacts', 'logs'); if ~exist(d, 'dir'), mkdir(d); end
logFile = fullfile(d, sprintf('rig_%s_%s.mat', TASK, datestr(now, 'yyyymmdd_HHMMSS')));
save(logFile, 'L', 'P', 'C');
fprintf('Log: %s\n', logFile);
if ~isempty(stopMsg), error('run_rig:stopped', '%s (log saved)', stopMsg); end
fprintf('PID: Kp %.2f V/rad, Ki %.2f V/(rad s), Kd %.3f V s/rad (plot: plot_pid_terms(''%s''))\n', C.Kp, C.Ki, C.Kd, logFile);
e = (L.ref(1,:) - L.y)*M.r*1e3;
fprintf('Carriage tracking error: RMS %.2f mm, max %.2f mm\n', sqrt(mean(e(~isnan(e)).^2)), max(abs(e)));
if useCam
    kf_ = ~isnan(L.glints) & L.srcCode == 2;     % frames with the eyes tracked
    if any(kf_)
        fprintf('IR glint (LED reflection) on the iris: >= 1 eye in %.0f %% of eye frames, both eyes in %.0f %%\n', ...
                100*mean(L.glints(kf_) >= 1), 100*mean(L.glints(kf_) == 2));
    end
    if any(~isnan(L.gazeDeg))
        fprintf('Gaze follow: angle %+.0f to %+.0f deg (P.gzf.on = %d)\n', min(L.gazeDeg), max(L.gazeDeg), P.gzf.on);
    end
end
if strcmp(TASK, 'step') || strcmp(TASK, 'step_camera')
    lv = L.tReal; ok_ = ~isnan(L.y);
    for j = 1:floor(max(lv(ok_))/P.stepTest.period)
        i = find(ok_ & lv < j*P.stepTest.period, 1, 'last');
        fprintf('step %d: error at the end %.2f mm\n', j, (L.ref(1, i) - L.y(i))*M.r*1e3);
    end
    if useCam, fprintf('Loop time with the camera: median %.0f ms\n', median_valid(L.loopMs)); end
end
if useCam && ~realtime                          % virtual: compare with the virtual user's head
    ok = ~isnan(L.head(1,:));
    S = synth_scene(P, 'demo'); hx = arrayfun(@(tt) S.at(tt).head(1), t(ok));
    eh = (hx - L.y(ok)*M.r)*1e3;
    eh = eh(t(ok) > 2);
    fprintf('Camera offset from the head (true, t > 2 s): RMS %.1f mm, max %.1f mm\n', sqrt(mean(eh.^2)), max(abs(eh)));
end
if SHOW
    plot_rig_log(L, P);
    if any(~isnan(L.uPID(1, :))), plot_pid_terms(logFile); end   % P, I, D terms (saved as PNG)
end
