function [img, truth] = synth_render(P, s, xCar, ledMask, tilt)
%SYNTH_RENDER  Synthetic OV9281 IR frame of a face (two eyes) for offline work.
%   s       : scene state from synth_scene(...).at(t)
%   xCar    : carriage (camera) position [m]
%   ledMask : 1 x P.led.n logical, LEDs switched on
%   tilt    : 1x2 servo LED tilt angles [rad] (beam direction, 0 = horizontal)
%   img     : H x W uint8 (mono)
%   truth   : ground truth: .pupil (2x2 full-image px, [right left]),
%             .glints{e} (2xN px), .rPupilPx (1x2), .visible (1x2), .illum
%   Model: dark pupil, iris, sclera, face lit by the LEDs (beam angle), one
%   glint per lit LED from exact spherical-cornea reflection, eyebrows and
%   nostrils as false dark blobs, eyelids closed during blinks, sensor noise.
%   Anti-aliased edges so sub-pixel accuracy can be tested.
if nargin < 4 || isempty(ledMask), ledMask = true(1, P.led.n); end
if nargin < 5 || isempty(tilt), tilt = [0 0]; end
W = P.cam.W; H = P.cam.H;
C = P.geo.cam + [xCar; 0; 0];
Rr = [cos(s.roll) -sin(s.roll) 0; sin(s.roll) cos(s.roll) 0; 0 0 1];
pxPerM = @(z) P.cam.f_px/max(z - C(3), 1e-3);

% ---- LED illumination of the face (beam pattern, cos^n-like)
Lpos = C + P.led.pos;
illum = 0; ledI = zeros(1, P.led.n);
for i = 1:P.led.n
    if ~ledMask(i), continue, end
    ax = [0; 0; 1];
    if P.led.servo(i) > 0, a = tilt(P.led.servo(i)); ax = [0; sin(a); cos(a)]; end
    d = s.head - Lpos(:,i); d = d/norm(d);
    ang = acos(max(-1, min(1, ax'*d)))*180/pi;
    ledI(i) = 2^(-(ang/P.led.halfAngleDeg)^2);
    illum = illum + ledI(i);
end
illum = min(1.3, 0.15 + 0.5*illum);

img = 35 + zeros(H, W);                                   % background
k = pxPerM(s.head(3));
[hu, hv] = proj(s.head + Rr*[0; -0.02; 0]);
img = ellipse(img, hu, hv, 0.075*k, 0.10*k, s.roll, 40 + 120*illum);   % face

truth.pupil = nan(2, 2); truth.glints = {zeros(2,0), zeros(2,0)};
truth.rPupilPx = [0 0]; truth.visible = [false false]; truth.illum = illum;
for e = 1:2
    ctr = s.eyes(:, e);
    ke = pxPerM(ctr(3));
    [eu, ev] = proj(ctr);
    if P.syn.distractors                                   % eyebrow
        [bu, bv] = proj(ctr + Rr*[0; 0.022; -0.005]);
        img = ellipse(img, bu, bv, 0.017*ke, 0.0035*ke, s.roll, 45 + 25*illum);
    end
    if s.blink
        img = ellipse(img, eu, ev, 0.014*ke, 0.005*ke, s.roll, 30 + 115*illum);  % eyelid
        continue
    end
    E = eye_model(P, ctr, s.target);
    img = ellipse(img, eu, ev, 0.014*ke, 0.0058*ke, s.roll, 60 + 140*illum);  % sclera
    [pu, pv] = proj(E.p);
    img = ellipse(img, pu, pv, 0.006*ke, 0.006*ke, 0, 40 + 70*illum);         % iris
    rp = 0.002*ke;
    img = ellipse(img, pu, pv, rp, rp, 0, 12);                                % pupil
    truth.pupil(:, e) = [pu; pv]; truth.rPupilPx(e) = rp; truth.visible(e) = true;
    for i = find(ledMask)
        if ledI(i) < 0.05 || ~P.syn.glints, continue, end
        G = glint_point(E.c, E.R, Lpos(:,i), C);
        [gu, gv] = proj(G);
        img = spot(img, gu, gv, 0.7, 255*min(1, 0.4 + ledI(i)));
        truth.glints{e}(:, end+1) = [gu; gv];
    end
end
if P.syn.distractors                                         % nostrils
    for sx = [-1 1]
        [nu, nv] = proj(s.head + Rr*[sx*0.009; -0.055; -0.02]);
        img = ellipse(img, nu, nv, 0.0045*k, 0.0028*k, s.roll + sx*0.3, 25);
    end
end
img = img + P.syn.noise*randn(H, W);
img = uint8(min(max(img, 0), 255));

    function [u, v] = proj(X)
        uv = cam_project(P, X, xCar); u = uv(1); v = uv(2);
    end
end

function img = ellipse(img, uc, vc, a, b, ang, val)
% anti-aliased filled ellipse (semi-axes a, b in px, rotation ang [rad], v down)
[H, W] = size(img);
R = max(a, b) + 2;
c1 = max(1, floor(uc - R)); c2 = min(W, ceil(uc + R));
r1 = max(1, floor(vc - R)); r2 = min(H, ceil(vc + R));
if c1 > c2 || r1 > r2, return, end
[X, Y] = meshgrid((c1:c2) - 0.5 - uc, (r1:r2) - 0.5 - vc);
ca = cos(ang); sa = sin(ang);
xr =  ca*X - sa*Y;                  % image v is down -> rotate with -ang visually
yr =  sa*X + ca*Y;
rho = sqrt((xr/a).^2 + (yr/b).^2);
alpha = min(1, max(0, (1 - rho)*min(a, b) + 0.5));
patch = img(r1:r2, c1:c2);
img(r1:r2, c1:c2) = patch.*(1 - alpha) + val*alpha;
end

function img = spot(img, uc, vc, sigma, val)
% bright Gaussian spot (glint), saturating
[H, W] = size(img);
c1 = max(1, floor(uc - 4)); c2 = min(W, ceil(uc + 4));
r1 = max(1, floor(vc - 4)); r2 = min(H, ceil(vc + 4));
if c1 > c2 || r1 > r2, return, end
[X, Y] = meshgrid((c1:c2) - 0.5 - uc, (r1:r2) - 0.5 - vc);
g = exp(-(X.^2 + Y.^2)/(2*sigma^2));
img(r1:r2, c1:c2) = max(img(r1:r2, c1:c2), val*g + img(r1:r2, c1:c2).*(1 - g));
end
