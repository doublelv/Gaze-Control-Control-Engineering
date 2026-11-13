function test_gaze_follow()
%TEST_GAZE_FOLLOW  gaze_follow.m (look, then move) in a small closed loop: a
%   synthetic eye whose apparent angle depends on where the camera is
%   (m = theta - B*phi; B = 1 for an eye alone, ~1.9 estimated on the rig), the
%   carriage following its reference (1 Hz, 0.2 m/s) with 0.1 s camera latency.
%   Checks (within the dead zone, ~5 cm): the camera ends at the point the eyes look at when B = 1 (face + gaze
%   and gaze only), and at the expected fixed point theta/B for B = 1.5, 1.9; eyes forward = in front of the head; eyes lost -> base;
%   oscillation guard; off switch.
P = params();
z = 0.45;
for B = [1 1.5 1.9]
    % face + gaze: base = head
    for c = [0 -0.15; -0.15 0; -0.15 atand(0.15/z); 0.15 15; -0.15 -15]'
        xh = c(1); th = c(2);
        [x, gz] = loop(P, B, xh, @(t) th, 14, NaN);
        want = xh + z*tand(th/(B - P.gzf.Bx));     % fixed point (= the gaze point when B = 1 + Bx)
        assert(strcmp(gz.phase, 'run'), 'B %.1f: phase %s', B, gz.phase);
        assert(abs(x(end) - want) < 0.05, 'B %.1f head %.0f mm gaze %.0f deg: camera %.0f mm, expected %.0f mm', ...
               B, xh*1e3, th, x(end)*1e3, want*1e3);
        if B == 1, assert(abs(x(end) - (xh + z*tand(th))) < 0.03, 'B 1: not at the gaze point'); end
    end
    % gaze only: base = rail centre (0); eyes right, then left
    [x, gz, tt] = loop(P, B, 0, @(t) 20*(t >= 4 & t < 12) - 20*(t >= 12), 20, 0);
    x1 = x(find(tt < 12, 1, 'last')); want = z*tand(20/(B - P.gzf.Bx));
    assert(abs(x1 - want) < 0.05 && abs(x(end) + want) < 0.05, 'B %.1f gaze only: %.0f / %.0f mm (want +-%.0f)', ...
           B, x1*1e3, x(end)*1e3, want*1e3);
    fprintf('B %.1f: gaze only +20 deg -> %+.0f mm, -20 deg -> %+.0f mm (gaze point %.0f mm)\n', B, x1*1e3, x(end)*1e3, z*tand(20)*1e3);
end

% eyes lost -> back to the base
[~, gz] = loop(P, 1.5, 0, @(t) 20, 8, NaN);
box = [400 200 200 200]; H = struct('z', z, 'pos', [0; 0; z]);
none = struct('seen', [false false], 'uv', nan(2));
for t = 8:0.05:10, gz = gaze_follow(gz, none, box, H, 0, t, false, P, true); end
assert(gz.psi == 0, 'eyes lost: theta %.1f', gz.psi);

% oscillation guard: eyes flipping left-right every 0.5 s
P2 = P; P2.gzf.nAvg = 1;
[~, gz] = loop(P2, 1.5, 0, @(t) 25*(t > 3)*sign(sin(2*pi*t)), 12, NaN, true);
assert(strcmp(gz.phase, 'off'), 'oscillation guard did not switch off (%s)', gz.phase);

% off
P.gzf.on = false;
[~, d] = gaze_follow([], struct('seen', [true true], 'uv', [500 300; 300 300]), box, H, 0, 0, true, P);
assert(isnan(d), 'gaze follow off: expected NaN');
end

function [x, gz, tt] = loop(P, B, xh, theta, T, xBase, alwaysSettled)
% carriage + synthetic eye; xBase NaN = base is the head (track_camera)
if nargin < 7, alwaysSettled = false; end
P.gzf.xBase = xBase;
f = P.cam.f_px; z = 0.45; rE = P.gzf.rEye;
box = [400 200 200 200]; cx = box(1) - 0.5 + box(3)/2; u0 = cx + [35 -35];
H = struct('z', z, 'pos', [xh; 0; z]);
base = xh; if ~isnan(xBase), base = xBase; end
dt = 0.045; tt = 0:dt:T; n = numel(tt); x = base*ones(1, n); ref = base; lat = round(0.1/dt); gz = [];
for k = 2:n
    xd = x(max(1, k - lat)); phi = atand((xd - xh)/z);
    th = theta(tt(k)); if tt(k) < 2.5, th = phi; end          % zeroing: looking at the camera
    m = th - phi - (B - 1)*phi;                                % apparent angle relative to the camera
    obs = struct('seen', [true true], 'uv', [u0 + rE*sind(m)*f/z; 300 300]);
    settled = alwaysSettled || abs(ref - x(k-1)) < P.gzf.settleMm*1e-3;
    [gz, d] = gaze_follow(gz, obs, box, H, xd, tt(k), true, P, settled);
    r_ = base; if ~isnan(d), r_ = base + z*tand(d); end
    r_ = min(max(r_, P.belt.xMin), P.belt.xMax);
    if abs(r_ - ref) >= P.trk.deadband, ref = r_; end
    x(k) = x(k-1) + min(max((ref - x(k-1))*2*pi, -0.2), 0.2)*dt;
end
end
