function test_tracker()
%TEST_TRACKER  Camera -> detector -> tracker -> head estimate on the synthetic
%   demo scene (moving head, roll, gaze changes, 2 blinks):
%   lock on the first frame, no false eye during blinks, head distance within
%   2 cm, lateral position within 1 cm, roll within 2 deg.
P = params(); P.cam.source = 'synthetic'; rng(12);
cam = camera_open(P);
T = eye_tracker_init(P);
ex = []; ez = []; er = []; nFalse = 0;
for t = 0:1/20:6
    xCar = 0.02*sin(2*pi*0.2*t);                       % camera moves too
    [cam, img, info] = camera_read(cam, P, struct('t', t, 'xCar', xCar));
    det = eye_detect(img, P, eye_tracker_prior(T, t, P));
    [T, obs] = eye_tracker_update(T, det, t, P);
    [H, T] = head_estimate(obs, T, xCar, P);
    s = info.truth.scene;
    if t == 0, assert(strcmp(obs.status, 'track'), 'no lock on the first frame'); end
    if s.blink
        nFalse = nFalse + any(obs.seen);
        continue
    end
    assert(any(obs.seen), 't=%.2f: eyes lost (status %s)', t, obs.status);
    pupMid = mean(info.truth.pupil, 2);
    if obs.pairSeen
        ex(end+1) = H.pos(1) - (xCar + (pupMid(1) - P.cam.W/2)*s.head(3)/P.cam.f_px); %#ok<AGROW>
        ez(end+1) = H.z - s.head(3);                                                   %#ok<AGROW>
        er(end+1) = (H.roll - s.roll)*180/pi;                                          %#ok<AGROW>
    end
end
camera_close(cam);
assert(nFalse == 0, '%d false eyes during blinks', nFalse);
assert(max(abs(ez)) < 0.02, 'distance error %.3f m', max(abs(ez)));
assert(max(abs(ex)) < 0.01, 'lateral error %.3f m', max(abs(ex)));
assert(max(abs(er)) < 2, 'roll error %.2f deg', max(abs(er)));
end
