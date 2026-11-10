function [gz, deg] = gaze_follow(gz, obs, box, H, t, eyesOk, P)
%GAZE_FOLLOW  Horizontal gaze angle from the iris position inside the face box.
%   Used by run_rig track_camera when P.gzf.on: the carriage then goes to the
%   point on the rail the eyes look at, x = x_head + z*tan(theta), instead of
%   only following the head. With the head still and the eyes turning, the
%   iris moves only a few pixels, so this makes eye movements clearly visible.
%
%   gz     : state ([] at the start),  obs : eye_tracker_update output
%   box    : face box [x y w h] ([] = none),  H : head_estimate output (H.z used)
%   t      : frame time [s],  eyesOk : the eyes (not only the face) are tracked
%   deg    : filtered gaze angle [deg], + = toward +x (MAX side); NaN = unknown
%            (zeroing, or eyes missing longer than P.gzf.holdT) -> follow the head
%   gz.label : text for the live view
%
%   Model (approximate, no per-user calibration):
%   1. Zeroing: for the first P.gzf.tZero s with both eyes seen the user looks
%      straight at the camera; the iris position of each eye relative to the face
%      box centre (in box widths) is stored as rel0.
%   2. Offset d = mean(rel - rel0) * box width [px] -> d*z/f [m] at the eye.
%   3. The iris centre turns on a circle of radius P.gzf.rEye around the eyeball
%      centre: theta = asin(d/rEye). Dead zone P.gzf.deadDeg (face box jitter),
%      limit P.gzf.maxDeg, low-pass P.gzf.tau.
%   A head turn (yaw) also moves the iris inside the face box and is read as gaze.
if isempty(gz)
    gz = struct('rel0', [NaN NaN], 'zs', zeros(0, 2), 't0', NaN, 'zeroed', false, ...
                'th', NaN, 'tLast', NaN, 'label', '');
end
deg = NaN;
if ~P.gzf.on, gz.label = ''; return, end
G = P.gzf;
valid = eyesOk && ~isempty(box) && any(obs.seen) && isfinite(H.z);
rel = [NaN NaN];
if valid
    cx = box(1) - 0.5 + box(3)/2;                 % box centre, same pixel convention as face_head
    for e = 1:2
        if obs.seen(e), rel(e) = (obs.uv(1, e) - cx)/box(3); end
    end
end

if ~gz.zeroed                                     % 1. zeroing
    if valid && all(obs.seen)
        if isnan(gz.t0), gz.t0 = t; end
        gz.zs(end+1, :) = rel;
    end
    if ~isnan(gz.t0) && t - gz.t0 >= G.tZero && size(gz.zs, 1) >= G.minSamples
        gz.rel0 = median(gz.zs, 1);
        gz.zeroed = true;
        gz.label = ' | gaze zeroed';
    else
        gz.label = ' | gaze: look at the camera (zeroing)';
    end
    return
end

if valid
    d = rel - gz.rel0; d = mean(d(~isnan(d)));    % 2. iris offset [box widths]
    dM = d*box(3)*H.z/P.cam.f_px;                 %    [m] at the eye
    s = min(max(dM/G.rEye, -sind(G.maxDeg)), sind(G.maxDeg));
    thRaw = asind(s);                             % 3. gaze angle
    thRaw = sign(thRaw)*max(0, abs(thRaw) - G.deadDeg);
    if isnan(gz.th) || isnan(gz.tLast)
        gz.th = thRaw;
    else
        a = min(1, max(t - gz.tLast, 0)/G.tau);
        gz.th = gz.th + a*(thRaw - gz.th);
    end
    gz.tLast = t;
elseif ~isnan(gz.tLast) && t - gz.tLast > G.holdT
    gz.th = NaN;                                  % eyes gone too long: follow the head
end
deg = gz.th;
if isnan(deg), gz.label = ' | gaze -'; else, gz.label = sprintf(' | gaze %+.0f deg', deg); end
end
