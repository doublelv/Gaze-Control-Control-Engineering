function [H, T] = head_estimate(obs, T, xCar, P)
%HEAD_ESTIMATE  Eye pixels -> 3-D head position (world frame, metres).
%   obs  : from eye_tracker_update,  T : tracker state (keeps the distance estimate)
%   xCar : carriage (camera) position [m] from the encoder
%   H.valid : a midpoint could be computed this frame
%   H.pos   : 3x1 midpoint between the eyes [m]
%   H.eyes  : 3x2 pupil positions [right left] [m] (NaN if not seen)
%   H.z     : distance [m] (from the pupil spacing and P.det.ipd, low-pass filtered)
%   H.roll  : head roll [rad] (+ = counter-clockwise seen by the camera), NaN if unknown
%   Camera model: pinhole, f = P.cam.f_px, principal point at the image centre.
H = struct('valid', false, 'pos', nan(3, 1), 'eyes', nan(3, 2), 'z', T.z, 'roll', NaN);
f = P.cam.f_px;
if obs.pairSeen && obs.sep > 1
    zNew = f*P.det.ipd/obs.sep;
    zNew = min(max(zNew, P.det.zRange(1)*0.7), P.det.zRange(2)*1.3);
    T.z = (1 - P.head.zAlpha)*T.z + P.head.zAlpha*zNew;
    d = obs.uv(:, 1) - obs.uv(:, 2);
    H.roll = atan2(-d(2), d(1));                 % image v points down
end
H.z = T.z;
C = P.geo.cam + [xCar; 0; 0];
toW = @(uv) C + [(uv(1) - P.cam.W/2)*H.z/f; -(uv(2) - P.cam.H/2)*H.z/f; H.z];
for e = 1:2
    if obs.seen(e), H.eyes(:, e) = toW(obs.uv(:, e)); end
end
if ~isnan(obs.mid(1))
    H.pos = toW(obs.mid);
    H.valid = true;
end
end
