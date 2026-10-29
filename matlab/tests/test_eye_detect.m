function test_eye_detect()
%TEST_EYE_DETECT  Detector on synthetic OV9281 frames (synth_render, no real recordings):
%   both pupils found in every open-eye frame, no false pair, error < 1 px,
%   clean anti-aliased disc centred to < 0.05 px, nothing found during a blink.
P = params(); rng(11);
S = synth_scene(P, 'demo');
err = [];
for t = 0.3:0.45:5.8
    s = S.at(t);
    [img, tr] = synth_render(P, s, s.head(1));
    d = eye_detect(img, P, []);
    if s.blink
        assert(~d.pair, 't=%.2f: pair found during a blink', t);
        continue
    end
    assert(d.pair && numel(d.eyes) == 2, 't=%.2f: eye pair not found', t);
    for e = 1:2
        err(end+1) = hypot(d.eyes(e).uv(1) - tr.pupil(1, e), d.eyes(e).uv(2) - tr.pupil(2, e)); %#ok<AGROW>
        assert(~isempty(d.eyes(e).glints), 't=%.2f: no glint', t);
    end
end
assert(max(err) < 1.0, 'max pupil error %.2f px', max(err));
assert(mean(err) < 0.5, 'mean pupil error %.2f px', mean(err));
% clean disc: sub-pixel accuracy of the centroid
[X, Y] = meshgrid((1:P.cam.W) - 0.5, (1:P.cam.H) - 0.5);
img = 190*ones(P.cam.H, P.cam.W); c = [640.37 360.81; 578.37 361.81];
for k = 1:2
    di = hypot(X - c(k,1), Y - c(k,2));
    a = min(1, max(0, 6 - di + 0.5)); img = img.*(1 - a) + 120*a;
    a = min(1, max(0, 1.8 - di + 0.5)); img = img.*(1 - a) + 12*a;
end
d = eye_detect(uint8(img), P, []);
assert(numel(d.eyes) == 2 && norm(d.eyes(1).uv - c(1,:)) < 0.05, 'clean disc not centred');
assert(abs(d.eyes(1).r - 1.8) < 0.2, 'radius %.2f', d.eyes(1).r);
% blink frame
s = S.at(P.cam.blinks(1) + 0.05);
d = eye_detect(synth_render(P, s, 0), P, []);
assert(numel(d.eyes) < 2 || ~d.pair, 'eyes reported during a blink');
end
