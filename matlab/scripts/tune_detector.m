%TUNE_DETECTOR  Check the pupil/glint detector on recorded frames and suggest
%   parameter changes. Run record_session first (or use SOURCE = 'synthetic').
%   Prints detection rate, pupil size, glint count, and shows a grid of frames
%   with the overlays. Hardware effects: none.

clear; close all;
[root, mroot] = setup_paths();
P = params();
SOURCE = 'synthetic';          % 'video' | 'synthetic' | 'images'
P.cam.videoFile = '';          % e.g. fullfile(mroot, 'data', 'local', 'session_....avi')
N_FRAMES = 60;

P.cam.source = SOURCE;
cam = camera_open(P);
T = eye_tracker_init(P);
st = struct('pair', 0, 'one', 0, 'none', 0, 'r', [], 'ng', [], 'cand', [], 'z', []);
shots = {};
for k = 1:N_FRAMES
    t = k/30;
    [cam, img, inf_] = camera_read(cam, P, struct('t', t));
    if ~inf_.ok, break, end
    det = eye_detect(img, P, eye_tracker_prior(T, t, P));
    [T, obs] = eye_tracker_update(T, det, t, P);
    [H, T] = head_estimate(obs, T, 0, P);
    st.cand(end+1) = det.nCand;
    if obs.pairSeen, st.pair = st.pair + 1; st.z(end+1) = H.z;
    elseif any(obs.seen), st.one = st.one + 1; else, st.none = st.none + 1; end
    for e = find(obs.seen)
        st.r(end+1) = obs.eye{e}.r; st.ng(end+1) = size(obs.eye{e}.glints, 1);
    end
    if mod(k, max(1, floor(N_FRAMES/6))) == 0 && numel(shots) < 6, shots{end+1} = {img, det, obs}; end %#ok<SAGROW>
end
n = st.pair + st.one + st.none;
fprintf('\nframes %d: both eyes %d (%.0f%%), one eye %d, none %d\n', n, st.pair, 100*st.pair/n, st.one, st.none);
fprintf('pupil radius: median %.2f px (5-95%%: %.2f-%.2f)\n', median(st.r), percentile_valid(st.r, 5), percentile_valid(st.r, 95));
fprintf('glints per eye: median %d, eyes without glint %.0f%%\n', median(st.ng), 100*mean(st.ng == 0));
fprintf('dark candidates per frame: median %d\n', median(st.cand));
if ~isempty(st.z), fprintf('eye distance from IPD: median %.2f m\n', median(st.z)); end
fprintf('\nSuggestions:\n');
if st.pair < 0.8*n
    fprintf(' - Eyes often missed. Check the overlays: if the pupil is not marked as a candidate (+),\n');
    fprintf('   raise P.det.absDark (now %d) or lower P.det.darkK (now %.2f) / P.det.darkBlobK (now %.2f).\n', ...
            P.det.absDark, P.det.darkK, P.det.darkBlobK);
end
if median(st.cand) > 15
    fprintf(' - Many dark candidates: lower P.det.absDark or raise P.det.minEncl (now %.2f).\n', P.det.minEncl);
end
if ~isempty(st.ng) && mean(st.ng == 0) > 0.2
    fprintf(' - Glints missing: lower P.det.glintMin (now %d) or check that the IR LEDs are on.\n', P.det.glintMin);
end
if ~isempty(st.r) && median(st.r) < 1.3
    fprintf(' - Pupils are very small (< 1.3 px): sit closer (40-60 cm).\n');
end
if ~isempty(st.z) && abs(median(st.z) - 0.6) > 0.25
    fprintf(' - Distance looks off: measure the user''s IPD and set P.det.ipd.\n');
end
figure('Name', 'tune_detector', 'Position', [50 50 1400 700]);
for k = 1:numel(shots)
    ax = subplot(2, 3, k);
    viz_detection(ax, shots{k}{1}, shots{k}{2}, shots{k}{3});
end
camera_close(cam);
