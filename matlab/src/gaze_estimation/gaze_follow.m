function [gz, deg] = gaze_follow(gz, obs, box, H, xCam, t, eyesOk, P)
%GAZE_FOLLOW  Move the camera toward where the eyes look (visual servoing on the
%   iris position). Used by run_rig track_camera when P.gzf.on: the carriage
%   reference becomes x_head + z*tan(psi) instead of x_head.
%
%   gz    : state ([] at the start),  obs : eye_tracker_update output
%   box   : face box [x y w h] ([] = none),  H : head_estimate output (H.pos, H.z)
%   xCam  : carriage (camera) x when the frame was taken [m]
%   t     : frame time [s],  eyesOk : the eyes (not only the face) are tracked
%   deg   : psi [deg], + = toward +x (MAX side); NaN = zeroing / off -> follow the head
%   gz.m, gz.phi : eye angle relative to the camera and camera angle [deg] (logged)
%
%   1. Zeroing (P.gzf.tZero s): the user looks AT THE CAMERA; rel0 = iris position
%      of each eye inside the face box (in box widths), phi0 = camera angle.
%   2. m = asin(d/rEye): how far the eyes look away from the camera
%      (d = iris offset from rel0 at the eye [m]). + = toward +x.
%   3. psi integrates m (gain P.gzf.Ki, dead zone P.gzf.deadDeg): while the eyes
%      look to one side of the camera, the camera keeps moving that way; it stops
%      when the eyes look at the camera again (m = 0). Because the measurement is
%      relative to the camera, the camera's own motion reduces m: negative
%      feedback, stable without knowing how strongly the camera motion changes
%      the apparent eye angle (B, ~1.9 in the 2026-11-10 step_camera log; a
%      direct "target = x_head + z*tan(m)" ran away on the rig for that reason).
%      m + P.gzf.Bcomp*(phi - phi0) is used: part of the camera's own effect is
%      removed so the camera travels further (keep Bcomp < 1: B >= 1 for an eye).
%   |m| > P.gzf.mMax is a wrong iris detection and is ignored.
%   psi is limited to the rail (no wind-up), held during blinks, and returns to 0
%   when the eyes are lost longer than P.gzf.lostT. Safety: P.gzf.oscFlips side
%   changes of psi within P.gzf.oscWin s -> off (head following only).
if isempty(gz)
    gz = struct('phase', 'wait', 'tPh', NaN, 'zs', zeros(0, 3), 'rel0', [NaN NaN], 'phi0', NaN, ...
                'psi', 0, 'tLast', NaN, 'tSeen', NaN, 'm', NaN, 'phi', NaN, 'label', '', ...
                'lastSign', 0, 'flips', zeros(1, 0));
end
deg = NaN; gz.m = NaN; gz.phi = NaN;
if ~P.gzf.on, gz.label = ''; return, end
G = P.gzf;
valid = eyesOk && ~isempty(box) && any(obs.seen) && isfinite(H.z) && ~isnan(H.pos(1));
rel = [NaN NaN];
if valid
    cx = box(1) - 0.5 + box(3)/2;                 % box centre, same pixel convention as face_head
    for e = 1:2
        if obs.seen(e), rel(e) = (obs.uv(1, e) - cx)/box(3); end
    end
    gz.phi = atand((xCam - H.pos(1))/H.z);
end
if ~any(isnan(gz.rel0)) && valid
    d = rel - gz.rel0; d = mean(d(~isnan(d)));
    s = d*box(3)*H.z/P.cam.f_px/G.rEye;
    if abs(s) < sind(G.mMax), gz.m = asind(s); end   % larger = wrong iris (eye can't turn that far)
end

switch gz.phase
    case 'wait'
        gz.label = ' | gaze: look AT THE CAMERA';
        if valid && all(obs.seen), gz.phase = 'zero'; gz.tPh = t; end
    case 'zero'
        gz.label = ' | gaze: zeroing - keep looking at the camera';
        if valid && all(obs.seen), gz.zs(end+1, :) = [rel gz.phi]; end
        if t - gz.tPh >= G.tZero && size(gz.zs, 1) >= G.minSamples
            gz.rel0 = median(gz.zs(:, 1:2), 1); gz.phi0 = median(gz.zs(:, 3));
            gz.phase = 'run'; gz.tLast = t; gz.tSeen = t;
            fprintf('Gaze follow: zeroed (%d frames). Now look left or right: the camera follows.\n', size(gz.zs, 1));
        end
    case 'run'
        dt = min(max(t - gz.tLast, 0), 0.2); gz.tLast = t;
        if valid && ~isnan(gz.m)
            gz.tSeen = t;
            e = gz.m + G.Bcomp*(gz.phi - gz.phi0);    % eyes relative to the camera
            e = sign(e)*max(0, abs(e) - G.deadDeg);
            gz.psi = gz.psi + G.Ki*e*dt;
        elseif t - gz.tSeen > G.lostT
            gz.psi = gz.psi*max(0, 1 - dt/G.lostTau); % eyes gone: back to the head
        end
        % stay on the rail (no wind-up) and within the angle limit
        if isfinite(H.z) && ~isnan(H.pos(1))
            lo = atand((P.belt.xMin - H.pos(1))/H.z); hi = atand((P.belt.xMax - H.pos(1))/H.z);
            gz.psi = min(max(gz.psi, lo), hi);
        end
        gz.psi = min(max(gz.psi, -G.maxDeg), G.maxDeg);
        % safety: psi swinging from one side to the other again and again -> off
        sg = sign(gz.psi)*(abs(gz.psi) > G.oscDeg);
        if sg ~= 0 && sg ~= gz.lastSign
            if gz.lastSign ~= 0, gz.flips(end+1) = t; end
            gz.lastSign = sg;
        end
        gz.flips = gz.flips(gz.flips > t - G.oscWin);
        if numel(gz.flips) >= G.oscFlips
            gz.phase = 'off';
            fprintf('Gaze follow switched OFF: the camera swung left-right %d times in %.0f s. Following the head only.\n', ...
                    numel(gz.flips), G.oscWin);
        else
            deg = gz.psi;
        end
        gz.label = sprintf(' | gaze %+.0f deg (eyes %+.0f)', gz.psi, gz.m);
    otherwise                                     % 'off'
        gz.label = ' | gaze off (head only)';
end
end
