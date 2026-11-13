function [gz, deg] = gaze_follow(gz, obs, box, H, xCam, t, eyesOk, P, settled)
%GAZE_FOLLOW  Where on the rail do the eyes look? Used by run_rig track_gaze and
%   track_camera: the carriage reference becomes x_base + z*tan(theta), with
%   x_base = the head (track_camera) or the rail centre (track_gaze, P.gzf.xBase).
%
%   gz      : state ([] at the start),  obs : eye_tracker_update output
%   box     : face box [x y w h] ([] = none),  H : head_estimate output (H.pos, H.z)
%   xCam    : carriage (camera) x when the frame was taken [m]
%   t       : frame time [s],  eyesOk : the eyes (not only the face) are tracked
%   settled : the carriage stands at its reference (default true)
%   deg     : gaze angle theta [deg] from straight ahead, + = toward +x (MAX side);
%             NaN = zeroing / off -> the caller uses its base position
%   gz.m, gz.phi : eye angle relative to the camera and camera angle [deg] (logged)
%
%   1. Zeroing (P.gzf.tZero s): the user looks AT THE CAMERA; rel0 = iris position
%      of each eye inside the face box (in box widths), phi0 = camera angle.
%   2. m = asin(d/rEye): angle between the gaze and the line to the camera
%      (d = iris offset from rel0 at the eye [m]); phi = angle of that line.
%      Gaze from straight ahead: theta = m + phi + Bx*(phi - phi0). Bx corrects the
%      extra shift of the face box when the camera looks from the side (rig logs:
%      rough estimates 0.3-1.9 times the camera angle in total, so Bx = 0 is used,
%      i.e. pure eye geometry; with a true total factor B the camera then settles at
%      theta/B, short of the gaze point when B > 1).
%   3. Look, then move: theta is averaged only while the carriage stands still
%      (P.gzf.nAvg frames); a new target is set when it differs by more than
%      P.gzf.deadDeg; it moves the target by P.gzf.alpha of the difference. The
%      carriage then moves and the next look starts when it has arrived. Each look
%      shrinks the remaining error by the factor (1 - alpha*(B - Bx)), so it settles
%      without swinging for 0 < B - Bx < 2/alpha, instead of making the camera chase
%      its own motion (a continuous version did that on the rig, 2026-11-10).
%   |m| > P.gzf.mMax = wrong iris and is ignored; theta is limited to the rail and
%   to +-P.gzf.maxDeg; eyes lost longer than P.gzf.lostT -> theta back to 0.
%   Safety: P.gzf.oscFlips side changes within P.gzf.oscWin s -> off.
if nargin < 9 || isempty(settled), settled = true; end
if isempty(gz)
    gz = struct('phase', 'wait', 'tPh', NaN, 'zs', zeros(0, 3), 'rel0', [NaN NaN], 'phi0', NaN, ...
                'psi', 0, 'tLast', NaN, 'tSeen', NaN, 'm', NaN, 'phi', NaN, 'label', '', 'buf', zeros(1, 0), ...
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
        gz.tLast = t;
        if valid && ~isnan(gz.m)
            gz.tSeen = t;
            if settled                            % look while the carriage stands
                gz.buf(end+1) = gz.m + gz.phi + G.Bx*(gz.phi - gz.phi0);
                if numel(gz.buf) >= G.nAvg
                    th = median(gz.buf); gz.buf = zeros(1, 0);
                    if abs(th) < G.deadDeg, th = 0; end
                    if abs(th - gz.psi) > G.deadDeg, gz.psi = gz.psi + G.alpha*(th - gz.psi); end   % new target (relaxed)
                end
            else
                gz.buf = zeros(1, 0);             % moving: the view is changing
            end
        elseif t - gz.tSeen > G.lostT
            gz.psi = 0; gz.buf = zeros(1, 0);     % eyes gone: back to the base
        end
        % stay on the rail and within the angle limit
        xb = H.pos(1);                            % the carriage moves from the head ...
        if isfield(G, 'xBase') && ~isnan(G.xBase), xb = G.xBase; end   % ... or from a fixed base (track_gaze)
        if isfinite(H.z) && ~isnan(xb)
            lo = atand((P.belt.xMin - xb)/H.z); hi = atand((P.belt.xMax - xb)/H.z);
            gz.psi = min(max(gz.psi, lo), hi);
        end
        gz.psi = min(max(gz.psi, -G.maxDeg), G.maxDeg);
        % safety: theta swinging from one side to the other again and again -> off
        sg = sign(gz.psi)*(abs(gz.psi) > G.oscDeg);
        if sg ~= 0 && sg ~= gz.lastSign
            if gz.lastSign ~= 0, gz.flips(end+1) = t; end
            gz.lastSign = sg;
        end
        gz.flips = gz.flips(gz.flips > t - G.oscWin);
        if numel(gz.flips) >= G.oscFlips
            gz.phase = 'off';
            fprintf('Gaze follow switched OFF: the target swung left-right %d times in %.0f s. Following the base only.\n', ...
                    numel(gz.flips), G.oscWin);
        else
            deg = gz.psi;
        end
        gz.label = sprintf(' | gaze %+.0f deg (eyes %+.0f)', gz.psi, gz.m);
    otherwise                                     % 'off'
        gz.label = ' | gaze off (head only)';
end
end
