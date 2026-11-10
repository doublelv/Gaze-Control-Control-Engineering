function test_gaze_follow()
%TEST_GAZE_FOLLOW  gaze_follow.m in a small closed loop: a synthetic eye whose
%   apparent angle depends on the camera position (m = thw - B*phi, B = 1.9 as
%   estimated on the rig, also B = 1 and 3), the carriage following the
%   reference with a first-order lag and 0.1 s camera latency.
%   Checks: zeroing, no motion when looking at the camera, motion toward the
%   gaze side and back, stable for several B, oscillation guard, blink hold,
%   off switch.
P = params();
f = P.cam.f_px; z = 0.45; rE = P.gzf.rEye;
box = [400 200 200 200]; cx = box(1) - 0.5 + box(3)/2;
u0 = cx + [35 -35];                              % iris columns when looking at the camera
eyes = @(thw, xc, B) struct('seen', [true true], ...
    'uv', [u0 + rE*sind(thw - B*atand(xc/z))*f/z; 300 300]);
H = struct('z', z, 'pos', [0; 0; z]);

for B = [1 1.9 3]
    [x, gz] = loop(P, eyes, H, B, @(t) 0, 6);
    assert(strcmp(gz.phase, 'run'), 'B %.1f: not running (%s)', B, gz.phase);
    assert(abs(x(end)) < 0.005, 'B %.1f: looking at the camera, carriage moved to %.0f mm', B, x(end)*1e3);
    % look 25 deg toward +x for 5 s, then -25 deg for 5 s
    [x, gz, tt] = loop(P, eyes, H, B, @(t) 25*(t >= 4 & t < 9) - 25*(t >= 9), 14);
    x1 = x(find(tt < 9, 1, 'last')); x2 = x(end);
    xEq = z*tand(min(P.gzf.maxDeg, (25 - P.gzf.deadDeg)/(B - P.gzf.Bcomp)));   % eyes look at the camera again (or limit)
    assert(x1 > 0.6*xEq && x1 < 1.4*xEq + 0.01, 'B %.1f: +25 deg -> %.0f mm (expected ~%.0f)', B, x1*1e3, xEq*1e3);
    assert(x2 < -0.6*xEq, 'B %.1f: -25 deg -> %.0f mm', B, x2*1e3);
    assert(strcmp(gz.phase, 'run'), 'B %.1f: switched off (%s)', B, gz.phase);
    s = sign(diff(x(tt > 4 & tt < 9))); s = s(s ~= 0);
    assert(sum(diff(s) ~= 0) <= 2, 'B %.1f: carriage oscillates (%d reversals)', B, sum(diff(s) ~= 0));
    fprintf('B %.1f: gaze +25 deg -> carriage %+.0f mm, -25 deg -> %+.0f mm\n', B, x1*1e3, x2*1e3);
end

% oscillation guard: eyes flipping fast from side to side -> off
P2 = P; P2.gzf.Ki = 20;
[~, gz] = loop(P2, eyes, H, 1.9, @(t) 15*(t > 3)*sign(sin(2*pi*1.0*t)), 12);
assert(strcmp(gz.phase, 'off'), 'oscillation guard did not switch off (%s)', gz.phase);

% blink: psi held, then back to 0 after P.gzf.lostT
[~, gz] = loop(P, eyes, H, 1.9, @(t) 25*(t >= 4), 7);
p0 = gz.psi; none = struct('seen', [false false], 'uv', nan(2));
gz = gaze_follow(gz, none, box, H, 0, 7.2, false, P);
assert(abs(gz.psi - p0) < 1e-9, 'blink: psi not held');
for t = 7.2:0.05:12, gz = gaze_follow(gz, none, box, H, 0, t, false, P); end
assert(abs(gz.psi) < 1, 'eyes lost: psi did not return to 0 (%.1f)', gz.psi);

% off
P.gzf.on = false;
[~, d] = gaze_follow([], eyes(25, 0, 1.9), box, H, 0, 0, true, P);
assert(isnan(d), 'gaze follow off: expected NaN');

    function [x, gz, tt] = loop(P, eyes, H, B, thw, T)
        dt = 0.045; tt = 0:dt:T; n = numel(tt);
        x = zeros(1, n); ref = 0; gz = [];
        lat = round(0.1/dt);
        for k = 2:n
            xd = x(max(1, k - lat));                 % camera position when the frame was taken
            [gz, d] = gaze_follow(gz, eyes(thw(tt(k)), xd, B), box, H, xd, tt(k), true, P);
            if ~isnan(d), r_ = H.pos(1) + H.z*tand(d); else, r_ = H.pos(1); end
            if abs(r_ - ref) >= P.trk.deadband, ref = r_; end
            v = min(max((ref - x(k-1))*2*pi*1.0, -0.2), 0.2);   % 1 Hz carriage, 0.2 m/s
            x(k) = x(k-1) + v*dt;
        end
    end
end
