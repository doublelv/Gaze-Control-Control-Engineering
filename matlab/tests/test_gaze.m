function test_gaze()
%TEST_GAZE  Synthetic 9-point calibration (headless) -> fit -> gaze on 12 new
%   targets. With the 1.65 mm lens at 0.6 m the PCCR vector spans only ~1.3 px
%   over the whole screen, so the bounds are in degrees, not pixels:
%   leave-one-out < 6 deg, median error on new targets < 7 deg.
P = params(); P.cam.source = 'synthetic'; rng(21);
cam = camera_open(P, synth_scene(P, 'calib', [0; 0.2; 0]));
[V, X, info] = gaze_calib_collect(P, cam, struct('show', false));
assert(all(info.n(:) >= P.gcal.minSamples), 'too few calibration samples');
M = gaze_calib_fit(V, X, P);
assert(all(M.nPts == 9), 'calibration points missing');
assert(max(M.loo) < 6, 'leave-one-out error %.1f deg', max(M.loo));
% save / load round trip (local, git-ignored folder)
P2 = P; P2.gcal.file = 'test_gaze_tmp.mat';
f = gaze_calib_file(P2, 'save', M); M2 = gaze_calib_file(P2, 'load'); delete(f);
assert(isequal(M2.coef, M.coef), 'save/load mismatch');
% new targets
Xt = screen_points(P, [4 3], 0.15);
T = eye_tracker_init(P); t = 50; err = [];
for k = 1:size(Xt, 2)
    cam.scene = synth_scene(P, 'calib', Xt(:, k));
    for j = 1:6
        t = t + 1/30;
        [cam, img, inf_] = camera_read(cam, P, struct('t', t, 'xCar', 0));
        det = eye_detect(img, P, eye_tracker_prior(T, t, P));
        [T, obs] = eye_tracker_update(T, det, t, P);
        [H, T] = head_estimate(obs, T, 0, P);
        G = gaze_calib_apply(M, gaze_features(obs, H, P), P);
        if G.valid && j > 2
            d = norm([Xt(:, k) - inf_.truth.scene.head]);
            err(end+1) = atan(norm(G.X - Xt(1:2, k))/d)*180/pi; %#ok<AGROW>
        end
    end
end
assert(numel(err) >= 40, 'too few gaze estimates');
assert(median(err) < 7, 'median gaze error %.1f deg', median(err));
end
