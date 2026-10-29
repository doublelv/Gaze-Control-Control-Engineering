function [V, X, info] = gaze_calib_collect(P, cam, ui, xCar)
%GAZE_CALIB_COLLECT  Show calibration targets and record PCCR features.
%   P   : params,  cam : camera_open(...) result
%   ui  : struct with .show (logical) and, if shown, .fig/.ax/.dot from
%         gaze_calib_window. With ui.show = false nothing is drawn (tests).
%   xCar: camera (carriage) position [m] during calibration (default 0)
%   V   : 2 x 2 x K median PCCR features [component, eye, target]
%   X   : 2 x K target positions on the screen plane [m]
%   info: per-target sample counts and spread (quality check)
%   For a 'synthetic' camera the virtual user looks at each target.
if nargin < 4, xCar = 0; end
[Xw, uvS] = screen_points(P);
K = size(Xw, 2);
X = Xw(1:2, :);
V = nan(2, 2, K);
info.n = zeros(2, K); info.spread = nan(2, K); info.uv = uvS;
T = eye_tracker_init(P);
synth = strcmp(cam.source, 'synthetic');
tSim = 0; dtSim = 1/30;
for k = 1:K
    if synth, cam.scene = synth_scene(P, 'calib', Xw(:, k)); end
    if ui.show, gaze_calib_window(ui, 'target', uvS(:, k)); end
    samples = {zeros(2, 0), zeros(2, 0)};
    t0 = tic; tRec = 0;
    while true
        if synth
            tSim = tSim + dtSim; tNow = tSim; tRec = tRec + dtSim;
            [cam, img, inf_] = camera_read(cam, P, struct('t', tSim, 'xCar', xCar));
        else
            [cam, img, inf_] = camera_read(cam, P);
            tNow = inf_.t; tRec = toc(t0);
        end
        if ~inf_.ok, break, end
        det = eye_detect(img, P, eye_tracker_prior(T, tNow, P));
        [T, obs] = eye_tracker_update(T, det, tNow, P);
        [H, T] = head_estimate(obs, T, xCar, P);
        if tRec > P.gcal.settle
            F = gaze_features(obs, H, P);
            for e = 1:2
                if F.valid(e), samples{e}(:, end+1) = F.v(:, e); end
            end
        end
        if ui.show, viz_drawnow(); end
        if tRec > P.gcal.settle + P.gcal.record, break, end
    end
    for e = 1:2
        s = samples{e};
        info.n(e, k) = size(s, 2);
        if size(s, 2) >= P.gcal.minSamples
            V(:, e, k) = median(s, 2);
            info.spread(e, k) = median(sqrt(sum((s - V(:, e, k)).^2, 1)));
        end
    end
end
if ui.show, gaze_calib_window(ui, 'done'); end
end
