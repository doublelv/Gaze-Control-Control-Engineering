function test_gaze_follow()
%TEST_GAZE_FOLLOW  gaze_follow.m on hand-made tracker outputs: zeroing, sign and
%   size of the angle, dead zone, blink hold, and the off switch.
P = params();
P.gzf.tau = 1e-3;                                % no low-pass: check the raw model
box = [400 200 200 200]; cx = box(1) - 0.5 + box(3)/2;
H = struct('z', 0.5);
u0 = cx + [35 -35];                              % iris columns when looking at the camera
obsAt = @(du) struct('seen', [true true], 'uv', [u0 + du; 300 300]);
gz = []; t = 0;
for k = 1:60                                     % 1. zeroing (looking at the camera)
    t = k*0.04; [gz, d] = gaze_follow(gz, obsAt(0), box, H, t, true, P);
end
assert(gz.zeroed, 'not zeroed after %.1f s', t);
[gz, d] = gaze_follow(gz, obsAt(0), box, H, t + 0.04, true, P); t = t + 0.04;
assert(d == 0, 'looking at the camera: %.2f deg', d);
% 2. iris 6 px toward +u: d = 6*0.5/550 = 5.45 mm -> asin(5.45/11) = 29.7 deg, minus the dead zone
du = 6; [gz, d] = gaze_follow(gz, obsAt(du), box, H, t + 0.04, true, P); t = t + 0.04;
th = asind(du*H.z/P.cam.f_px/P.gzf.rEye) - P.gzf.deadDeg;
assert(abs(d - th) < 0.1, 'angle %.2f, expected %.2f', d, th);
[gz, d] = gaze_follow(gz, obsAt(-du), box, H, t + 0.04, true, P); t = t + 0.04;
assert(abs(d + th) < 0.1, 'mirror angle %.2f, expected %.2f', d, -th);
% the whole face (box + irises) moving does not change the angle
box2 = box + [40 0 0 0];
o2 = obsAt(0); o2.uv(1, :) = o2.uv(1, :) + 40;
[gz, d] = gaze_follow(gz, o2, box2, H, t + 0.04, true, P); t = t + 0.04;
assert(d == 0, 'head moved, eyes straight: %.2f deg', d);
% 3. small jitter (1 px) stays in the dead zone
[gz, d] = gaze_follow(gz, obsAt(1), box, H, t + 0.04, true, P); t = t + 0.04;
assert(d == 0, 'jitter: %.2f deg', d);
% 4. blink: last angle kept for P.gzf.holdT, then NaN (follow the head)
[gz, d] = gaze_follow(gz, obsAt(du), box, H, t + 0.04, true, P); t = t + 0.04;
none = struct('seen', [false false], 'uv', nan(2));
[gz, d1] = gaze_follow(gz, none, box, H, t + 0.2, false, P);
assert(abs(d1 - th) < 0.1, 'blink: angle not held (%.2f)', d1);
[gz, d2] = gaze_follow(gz, none, box, H, t + P.gzf.holdT + 0.1, false, P);
assert(isnan(d2), 'eyes gone: expected NaN, got %.2f', d2);
% 5. off
P.gzf.on = false;
[~, d] = gaze_follow([], obsAt(du), box, H, 0, true, P);
assert(isnan(d), 'gaze follow off: expected NaN');
end
