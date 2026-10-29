function G = gaze_sim(P)
%GAZE_SIM  Monte-Carlo test of 3-D gaze (optical axis) reconstruction.
%
%   Synthetic eye: spherical cornea (radius R, centre c), pupil centre
%   p = c + K*g where g is the true optical axis.  Glints are computed by
%   exact reflection on the sphere, projected into a pinhole camera, then
%   Gaussian pixel noise is added.
%
%   Two reconstruction methods are compared:
%   'stereo' - 2 cameras + 2 lights, in the spirit of Shih & Liu (2004):
%              for every (camera, light) pair the cornea centre lies in the
%              plane through camera centre, light and glint ray; all planes
%              are intersected by least squares, the pupil is triangulated.
%              Needs no per-user R or K.
%   'mono'   - our rig (1 camera + 2 lights): the two planes give a line
%              through the camera; the depth along it is found from the
%              reflection law with an assumed cornea radius R, the pupil
%              from an assumed distance K.
%
%   [INFERRED - NOT CHECKED AGAINST THE PAPER]  The plane construction is the
%   standard spherical-cornea glint geometry; it was written from general
%   knowledge, not verified line-by-line against Shih & Liu's equations.
%   Simplifications: corneal refraction of the pupil is ignored and only the
%   optical axis is estimated (the visual axis needs a per-user kappa
%   calibration, ~5 deg).

rng(P.gaze.seed);
O1 = P.geo.cam; O2 = P.geo.cam2;
L  = [P.geo.illumA P.geo.illumB];
R  = P.eye.R; K = P.eye.K; f = P.cam.f_px;

cam1 = make_cam(O1, P.traj.p0, f);
cam2 = make_cam(O2, P.traj.p0, f);

% geometry sanity check: camera and lights collinear -> mono degenerate
nrm = cross(L(:,1) - O1, L(:,2) - O1);
G.collinearity = norm(nrm)/(norm(L(:,1) - O1)*norm(L(:,2) - O1));   % 0 = collinear

sig = P.gaze.sigmaPx; nT = P.gaze.nTrials;
G.sigma = sig;
G.errMono   = NaN(numel(sig), nT);
G.errStereo = NaN(numel(sig), nT);
G.depthErrMono = NaN(numel(sig), nT);

for a = 1:numel(sig)
    for tr = 1:nT
        % random eye position around nominal, random fixation target on z = 0
        c   = P.traj.p0 + [0.02; 0.02; 0.03].*(2*rand(3,1) - 1);
        tgt = [0.20*(2*rand - 1); 0.15*(2*rand - 1); 0];
        g   = (tgt - c)/norm(tgt - c);
        p   = c + K*g;

        cams = {cam1, cam2};
        uvG = cell(2,2); uvP = cell(2,1);
        for j = 1:2
            for li = 1:2
                Pg = glint_point(c, R, L(:,li), cams{j}.O);
                uvG{j,li} = project(cams{j}, Pg) + sig(a)*randn(2,1);
            end
            uvP{j} = project(cams{j}, p) + sig(a)*randn(2,1);
        end

        % ---- stereo ----
        A = zeros(4,3); b = zeros(4,1); r = 0;
        for j = 1:2
            for li = 1:2
                r = r + 1;
                d = backproject(cams{j}, uvG{j,li});
                nvec = cross(L(:,li) - cams{j}.O, d);
                nvec = nvec/norm(nvec);
                A(r,:) = nvec'; b(r) = nvec'*cams{j}.O;
            end
        end
        cS = A\b;
        pS = triangulate(cam1.O, backproject(cam1, uvP{1}), cam2.O, backproject(cam2, uvP{2}));
        gS = (pS - cS)/norm(pS - cS);
        G.errStereo(a,tr) = angdeg(gS, g);

        % ---- mono (camera 1 only) ----
        [cM, pM] = recon_mono(cam1, L, uvG(1,:), uvP{1}, R, K);
        gM = (pM - cM)/norm(pM - cM);
        G.errMono(a,tr) = angdeg(gM, g);
        G.depthErrMono(a,tr) = norm(cM - c);
    end
end
end

% ======================================================================
function cam = make_cam(O, lookAt, f)
zc = (lookAt - O)/norm(lookAt - O);
xc = cross([0;1;0], zc); xc = xc/norm(xc);
yc = cross(zc, xc);
cam.O = O; cam.Rwc = [xc yc zc]; cam.f = f;
end

function uv = project(cam, X)
Xc = cam.Rwc'*(X - cam.O);
uv = cam.f*Xc(1:2)/Xc(3);
end

function d = backproject(cam, uv)
d = cam.Rwc*[uv/cam.f; 1];
d = d/norm(d);
end

function Pg = glint_point(c, R, Lp, O)
% Point on the sphere where light from Lp reflects into camera O.
u1 = (O - c)/norm(O - c);
v  = Lp - c; v = v - (v'*u1)*u1; u2 = v/norm(v);
phiL = atan2((Lp - c)'*u2, (Lp - c)'*u1);
fun = @(phi) refl_res(phi, c, R, Lp, O, u1, u2);
phi = fzero(fun, [0 phiL]);
Pg = c + R*(cos(phi)*u1 + sin(phi)*u2);
end

function r = refl_res(phi, c, R, Lp, O, u1, u2)
n  = cos(phi)*u1 + sin(phi)*u2;
Pp = c + R*n;
r  = n'*(O - Pp)/norm(O - Pp) - n'*(Lp - Pp)/norm(Lp - Pp);
end

function X = triangulate(O1, d1, O2, d2)
% midpoint of the shortest segment between two rays
A = [d1 -d2]; s = A\(O2 - O1);
X = 0.5*((O1 + s(1)*d1) + (O2 + s(2)*d2));
end

function [c, p] = recon_mono(cam, L, uvG, uvP, R, K)
O = cam.O;
gr = cell(1,2); nv = cell(1,2);
for li = 1:2
    gr{li} = backproject(cam, uvG{li});
    nv{li} = cross(L(:,li) - O, gr{li});
end
d = cross(nv{1}, nv{2}); d = d/norm(d);
if d'*gr{1} < 0, d = -d; end
opts = optimset('TolX', 1e-8);
tBest = fminbnd(@(tt) mono_res(tt, O, d, gr, L, R), 0.2, 1.5, opts);
c = O + tBest*d;
p = ray_sphere(O, backproject(cam, uvP), c, K);
end

function r = mono_res(tt, O, d, gr, L, R)
c = O + tt*d; r = 0;
for li = 1:2
    [Pp, miss] = ray_sphere(O, gr{li}, c, R);
    n  = (Pp - c)/norm(Pp - c);
    bs = (L(:,li) - Pp)/norm(L(:,li) - Pp) + (O - Pp)/norm(O - Pp);
    bs = bs/norm(bs);
    r = r + norm(n - bs)^2 + 10*miss^2;
end
end

function [X, miss] = ray_sphere(O, d, c, rad)
% nearest intersection of ray O + s d with sphere (c, rad);
% if it misses, return the closest point and the miss distance (relative)
w = O - c; bq = d'*w; cq = w'*w - rad^2;
disc = bq^2 - cq;
if disc >= 0
    s = -bq - sqrt(disc); X = O + s*d; miss = 0;
else
    s = -bq; X = O + s*d; miss = (norm(X - c) - rad)/rad;
end
end

function a = angdeg(u, v)
a = acos(max(-1, min(1, u'*v/(norm(u)*norm(v)))))*180/pi;
end
