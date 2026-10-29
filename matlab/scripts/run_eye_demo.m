%RUN_EYE_DEMO  Eye + head tracking with the camera only (no rig, no actuators).
%   Shows the camera image with the detected pupils and glints, eye close-ups,
%   head position / distance / roll and, if a gaze calibration exists
%   (run_calibration), the gaze point on the screen.
%
%   SOURCE: 'webcam'    the OV9281 on USB (P.cam.deviceName in params_local.m)
%           'synthetic' rendered face, no hardware at all
%           'video'     a recording from record_session (set VIDEO_FILE)
%   Close the figure window or press Ctrl-C to stop. Hardware effects: none.

clear; close all;
root = setup_paths();
P = params();

SOURCE     = 'synthetic';       % 'webcam' | 'synthetic' | 'video'
VIDEO_FILE = '';                % for 'video'
DURATION   = 30;                % [s], Inf = until the window is closed
SAVE_LOG   = true;              % head/gaze log to artifacts/logs (no images)

P.cam.source = SOURCE;
if strcmp(SOURCE, 'video'), P.cam.videoFile = VIDEO_FILE; end
cam = camera_open(P);
fprintf('Camera: %s\n', cam.name);
cleanupCam = onCleanup(@() camera_close(cam));
gcal = gaze_calib_file(P, 'load');
if isempty(gcal), fprintf('No gaze calibration found (run_calibration) - head/eye tracking only.\n');
else, fprintf('Gaze calibration from %s (LOO %.1f / %.1f deg)\n', gcal.created, gcal.loo); end

V = viz_live_init(P, 'Eye tracking demo');
T = eye_tracker_init(P);
kf = []; Gs = [];
n = 0; tPrev = NaN;
Lg = struct('t', [], 'head', zeros(3,0), 'z', [], 'roll', [], 'status', {{}}, 'gaze', zeros(2,0), 'ms', []);
t0 = tic;
while viz_alive(V.fig)
    tw = toc(t0);
    if tw > DURATION, break, end
    if strcmp(SOURCE, 'synthetic'), tf = n/30; else, tf = tw; end
    [cam, img, inf_] = camera_read(cam, P, struct('t', tf, 'xCar', 0));
    if ~inf_.ok, fprintf('Camera: %s\n', inf_.err); break, end
    n = n + 1;
    tp = tic;
    det = eye_detect(img, P, eye_tracker_prior(T, tf, P));
    [T, obs] = eye_tracker_update(T, det, tf, P);
    [H, T] = head_estimate(obs, T, 0, P);
    dt = tf - tPrev; if isnan(dt), dt = 1/30; end; tPrev = tf;
    [kf, pHat] = eye_filter(kf, H.pos, dt, P, H.valid);
    G = [];
    if ~isempty(gcal)
        G = gaze_calib_apply(gcal, gaze_features(obs, H, P), P);
        if G.valid                                      % light smoothing of the gaze point
            if isempty(Gs), Gs = G.uv; else, Gs = (1 - P.gcal.smooth)*G.uv + P.gcal.smooth*Gs; end
            G.uv = Gs; G.px = Gs.*P.screen.res(:);
        end
    end
    ms = toc(tp)*1e3;
    V = viz_live_update(V, tf, img, det, obs, H, G, NaN, struct('fps', round(n/max(tw, 1e-3)), 'proc_ms', round(ms)));
    Lg.t(end+1) = tf; Lg.head(:, end+1) = H.pos; Lg.z(end+1) = H.z; Lg.roll(end+1) = H.roll;
    Lg.status{end+1} = obs.status; Lg.ms(end+1) = ms;
    if ~isempty(G) && G.valid, Lg.gaze(:, end+1) = G.uv; else, Lg.gaze(:, end+1) = NaN; end
end
fprintf('%d frames, %.1f fps, processing %.1f ms/frame (median), tracker locks: %d\n', ...
        n, n/toc(t0), median(Lg.ms), T.nLocks);
if SAVE_LOG
    d = fullfile(root, 'artifacts', 'logs'); if ~exist(d, 'dir'), mkdir(d); end
    f = fullfile(d, sprintf('eye_demo_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
    save(f, 'Lg', 'P'); fprintf('Log: %s\n', f);
end
