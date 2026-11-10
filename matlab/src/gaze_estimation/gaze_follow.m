function [gz, deg] = gaze_follow(gz, obs, box, H, xCam, t, eyesOk, P)
%GAZE_FOLLOW  Horizontal gaze angle from the iris position inside the face box,
%   with a short start-up calibration. Used by run_rig track_camera (P.gzf.on):
%   the carriage then goes to the point on the rail the eyes look at,
%   x = x_head + z*tan(theta).
%
%   gz    : state ([] at the start),  obs : eye_tracker_update output
%   box   : face box [x y w h] ([] = none),  H : head_estimate output (H.pos, H.z)
%   xCam  : carriage (camera) x when the frame was taken [m]
%   t     : frame time [s],  eyesOk : the eyes (not only the face) are tracked
%   deg   : gaze angle in the world [deg], + = toward +x (MAX side);
%           NaN = not available (calibrating, failed, eyes lost) -> follow the head
%   gz.calRef : carriage position the calibration wants, relative to the rail
%           centre [m] (NaN = no request; run_rig then follows the head/gaze)
%   gz.m, gz.phi : raw iris angle and camera angle of this frame (logged)
%
%   Why a calibration: the iris offset is measured relative to the CAMERA. When
%   the camera moves sideways by the angle phi = atan((x_cam - x_head)/z), the eyes
%   look turned by about -phi even if they do not move, and the face box shifts
%   too. Measured on the rig (step_camera, 2026-11-10): m ~ -1.9*phi. Without a
%   correction the carriage chases its own motion (left-right oscillation).
%   Sequence (the user looks at ONE fixed point straight ahead the whole time):
%   1. zero  (P.gzf.tZero s, carriage at the rail centre): rel0 = iris position
%      inside the face box, phi0 = camera angle.
%   2. cal   (carriage to each P.gzf.calPts, P.gzf.calDwell s each, samples in the
%      last P.gzf.calRec s): fit m = -B*(phi - phi0) through the origin.
%      B outside P.gzf.Brange or a poor fit (R^2 < P.gzf.minR2) -> gaze follow off.
%   3. run   theta = m + B*(phi - phi0) with B = P.gzf.Bshrink * fitted B (a too
%      large B would be positive feedback), dead zone, limit, low-pass.
%      Safety: P.gzf.oscFlips side changes of theta within P.gzf.oscWin s -> off.
%   m = asin(d/rEye), d = iris offset at the eye [m] (P.gzf.rEye scales the gain).
if isempty(gz)
    gz = struct('phase', 'wait', 'tPh', NaN, 'zs', zeros(0, 3), 'cs', zeros(0, 2), ...
                'rel0', [NaN NaN], 'phi0', NaN, 'B', NaN, 'R2', NaN, 'th', NaN, 'tLast', NaN, ...
                'calRef', NaN, 'm', NaN, 'phi', NaN, 'label', '', 'Bfit', NaN, ...
                'lastSign', 0, 'flips', zeros(1, 0));
end
deg = NaN; gz.m = NaN; gz.phi = NaN;
if ~P.gzf.on, gz.label = ''; gz.calRef = NaN; return, end
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
if ~any(isnan(gz.rel0)) && valid                  % raw iris angle (relative to the camera)
    d = rel - gz.rel0; d = mean(d(~isnan(d)));
    s = min(max(d*box(3)*H.z/P.cam.f_px/G.rEye, -1), 1);
    gz.m = asind(s);
end

switch gz.phase
    case 'wait'                                   % until both eyes are seen
        gz.calRef = 0;
        gz.label = ' | gaze: look straight ahead at one point';
        if valid && all(obs.seen), gz.phase = 'zero'; gz.tPh = t; end
    case 'zero'
        gz.calRef = 0;
        gz.label = ' | gaze: zeroing - look straight ahead, keep still';
        if valid && all(obs.seen), gz.zs(end+1, :) = [rel gz.phi]; end
        if t - gz.tPh >= G.tZero && size(gz.zs, 1) >= G.minSamples
            gz.rel0 = median(gz.zs(:, 1:2), 1); gz.phi0 = median(gz.zs(:, 3));
            gz.phase = 'cal'; gz.tPh = t;
        end
    case 'cal'
        i = floor((t - gz.tPh)/G.calDwell) + 1;
        if i <= numel(G.calPts)
            gz.calRef = G.calPts(i);
            gz.label = sprintf(' | gaze calib %d/%d: keep looking at the SAME point, do not follow the camera', ...
                               i, numel(G.calPts));
            tIn = t - gz.tPh - (i - 1)*G.calDwell;
            if tIn > G.calDwell - G.calRec && ~isnan(gz.m)
                gz.cs(end+1, :) = [gz.phi - gz.phi0, gz.m];
            end
        else                                      % fit m = -B*dphi
            gz.calRef = NaN;
            x = gz.cs(:, 1); y = gz.cs(:, 2);
            if numel(x) >= G.minSamples && sum(x.^2) > 0
                gz.Bfit = -sum(x.*y)/sum(x.^2);
                gz.R2 = 1 - sum((y + gz.Bfit*x).^2)/max(sum((y - mean(y)).^2), eps);
            end
            if gz.Bfit >= G.Brange(1) && gz.Bfit <= G.Brange(2) && gz.R2 >= G.minR2
                gz.phase = 'run';
                gz.B = G.Bshrink*gz.Bfit;             % too large a B = positive feedback: stay below
                gz.label = sprintf(' | gaze calibrated (B %.2f, R2 %.2f)', gz.Bfit, gz.R2);
                fprintf('Gaze follow calibrated: B = %.2f (camera angle -> apparent eye angle, used %.2f), R^2 = %.2f, %d samples\n', ...
                        gz.Bfit, gz.B, gz.R2, numel(x));
            else
                gz.phase = 'off';
                fprintf(['Gaze follow calibration FAILED (B = %.2f, R^2 = %.2f, %d samples): following the head only. ', ...
                         'Look at one fixed point during the calibration and keep the head still.\n'], gz.Bfit, gz.R2, numel(x));
            end
        end
    case 'run'
        gz.calRef = NaN;
        if valid && ~isnan(gz.m)
            thRaw = gz.m + gz.B*(gz.phi - gz.phi0);   % world angle: own camera motion removed
            thRaw = sign(thRaw)*max(0, abs(thRaw) - G.deadDeg);
            thRaw = min(max(thRaw, -G.maxDeg), G.maxDeg);
            if isnan(gz.th) || isnan(gz.tLast)
                gz.th = thRaw;
            else
                a = min(1, max(t - gz.tLast, 0)/G.tau);
                gz.th = gz.th + a*(thRaw - gz.th);
            end
            gz.tLast = t;
            % safety: the angle swinging from one side to the other again and again
            % = the carriage chases its own motion -> switch gaze follow off
            sg = sign(gz.th)*(abs(gz.th) > G.oscDeg);
            if sg ~= 0 && sg ~= gz.lastSign
                if gz.lastSign ~= 0, gz.flips(end+1) = t; end
                gz.lastSign = sg;
            end
            gz.flips = gz.flips(gz.flips > t - G.oscWin);
            if numel(gz.flips) >= G.oscFlips
                gz.phase = 'off'; gz.th = NaN;
                fprintf('Gaze follow switched OFF: the angle swung left-right %d times in %.0f s (oscillation). Following the head only.\n', ...
                        numel(gz.flips), G.oscWin);
            end
        elseif ~isnan(gz.tLast) && t - gz.tLast > G.holdT
            gz.th = NaN;                              % eyes gone too long: follow the head
        end
        deg = gz.th;
        if isnan(deg), gz.label = ' | gaze -'; else, gz.label = sprintf(' | gaze %+.0f deg', deg); end
    otherwise                                     % 'off'
        gz.calRef = NaN;
        gz.label = ' | gaze off (head only)';
end
end
