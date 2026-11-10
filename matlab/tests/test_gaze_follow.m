function test_gaze_follow()
%TEST_GAZE_FOLLOW  gaze_follow.m on hand-made tracker outputs: zeroing, the
%   calibration of the camera-motion effect B, compensation, sign and size of
%   the angle, dead zone, blink hold, failed calibration and the off switch.
P = params();
P.gzf.tau = 1e-3;                                % no low-pass: check the raw model
P.gzf.Bshrink = 1;                               % exact compensation (rig default 0.8)
f = P.cam.f_px; z = 0.5; rE = P.gzf.rEye; Btrue = 1.9;
box = [400 200 200 200]; cx = box(1) - 0.5 + box(3)/2;
u0 = cx + [35 -35];                              % iris columns when looking straight ahead
% synthetic eye: world gaze thw [deg], camera at xc, head at 0
%   apparent angle m = thw - Btrue*phi  ->  iris offset d = rE*sind(m) at the eye
obsAt = @(thw, xc) struct('seen', [true true], ...
    'uv', [u0 + rE*sind(thw - Btrue*atand(xc/z))*f/z; 300 300]);
H = struct('z', z, 'pos', [0; 0; z]);
gz = []; t = 0; xc = 0; dt = 0.04; calRefs = [];
% 1.+2. zeroing and calibration: the user looks straight ahead (thw = 0) and the
%       carriage goes where gaze_follow asks (calRef)
while t < 15 && (isempty(gz) || any(strcmp(gz.phase, {'wait', 'zero', 'cal'})))
    t = t + dt;
    [gz, d] = gaze_follow(gz, obsAt(0, xc), box, H, xc, t, true, P);
    if ~isnan(gz.calRef), xc = gz.calRef; calRefs(end+1) = gz.calRef; end %#ok<AGROW>
    assert(isnan(d), 'angle during the calibration');
end
assert(strcmp(gz.phase, 'run'), 'calibration did not finish (phase %s)', gz.phase);
assert(any(calRefs < 0) && any(calRefs > 0), 'calibration did not move the carriage both ways');
assert(abs(gz.Bfit - Btrue) < 0.1, 'B %.2f, expected %.2f', gz.Bfit, Btrue);
% 3. eyes straight ahead, camera anywhere -> 0 (own motion removed)
for xc = [-0.08 0 0.05]
    t = t + dt; [gz, d] = gaze_follow(gz, obsAt(0, xc), box, H, xc, t, true, P);
    assert(abs(d) < 0.5, 'camera at %.0f mm, eyes straight: %.2f deg', xc*1e3, d);
end
% world gaze 25 deg -> 25 - dead zone, at two camera positions
for xc = [0 0.06]
    t = t + dt; [gz, d] = gaze_follow(gz, obsAt(25, xc), box, H, xc, t, true, P);
    assert(abs(d - (25 - P.gzf.deadDeg)) < 0.6, 'gaze 25 deg, camera %.0f mm: %.2f', xc*1e3, d);
end
t = t + dt; [gz, d] = gaze_follow(gz, obsAt(-25, 0), box, H, 0, t, true, P);
assert(abs(d + (25 - P.gzf.deadDeg)) < 0.6, 'gaze -25 deg: %.2f', d);
% small angle stays in the dead zone
t = t + dt; [gz, d] = gaze_follow(gz, obsAt(3, 0), box, H, 0, t, true, P);
assert(d == 0, 'small angle: %.2f deg', d);
% 4. blink: last angle kept for P.gzf.holdT, then NaN (follow the head)
t = t + dt; [gz, d0] = gaze_follow(gz, obsAt(25, 0), box, H, 0, t, true, P);
none = struct('seen', [false false], 'uv', nan(2));
[gz, d1] = gaze_follow(gz, none, box, H, 0, t + 0.2, false, P);
assert(abs(d1 - d0) < 1e-9, 'blink: angle not held');
[gz, d2] = gaze_follow(gz, none, box, H, 0, t + P.gzf.holdT + 0.1, false, P);
assert(isnan(d2), 'eyes gone: expected NaN, got %.2f', d2);
% oscillation guard: the angle swinging left-right fast -> off
gz2 = gz; t2 = t;
for j = 1:10
    t2 = t2 + 0.3; gz2 = gaze_follow(gz2, obsAt(30*(-1)^j, 0), box, H, 0, t2, true, P);
end
assert(strcmp(gz2.phase, 'off'), 'oscillation guard did not switch off');
% 5. calibration with random eye movements -> poor fit -> off (follow the head)
rng(1); gz = []; t = 0; xc = 0;
while t < 15 && (isempty(gz) || any(strcmp(gz.phase, {'wait', 'zero', 'cal'})))
    t = t + dt;
    thw = 0; if strcmp(gaze_phase(gz), 'cal'), thw = 30*randn; end
    gz = gaze_follow(gz, obsAt(thw, xc), box, H, xc, t, true, P);
    if ~isnan(gz.calRef), xc = gz.calRef; end
end
assert(strcmp(gz.phase, 'off'), 'noisy calibration: expected off, got %s', gz.phase);
% 6. off switch
P.gzf.on = false;
[~, d] = gaze_follow([], obsAt(25, 0), box, H, 0, 0, true, P);
assert(isnan(d), 'gaze follow off: expected NaN');
end

function p = gaze_phase(gz)
if isempty(gz), p = ''; else, p = gz.phase; end
end
