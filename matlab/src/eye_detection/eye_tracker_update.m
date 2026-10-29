function [T, obs] = eye_tracker_update(T, det, t, P)
%EYE_TRACKER_UPDATE  Associate detections with the right/left eye and update the state.
%   det : output of eye_detect,  t : frame time [s]
%   obs.status : tracker status after this frame
%   obs.uv     : 2x2 measured pupils [right left] this frame (NaN = not seen)
%   obs.eye    : 1x2 cell with the detection struct of each eye ([] = not seen)
%   obs.seen   : 1x2 logical,  obs.pairSeen : both eyes seen as a pair
%   obs.mid    : 2x1 midpoint between the eyes [px] (from one eye + offset if needed)
%   obs.sep    : pupil spacing [px] (NaN if not both seen)
T.nFrames = T.nFrames + 1;
dt = t - T.tLast; if isnan(dt) || dt <= 0, dt = 1/P.cam.fps; end
pred = T.uv + T.vel*min(dt, 0.2);
obs = struct('status', '', 'uv', nan(2, 2), 'eye', {{[], []}}, 'seen', [false false], ...
             'pairSeen', false, 'mid', [NaN; NaN], 'sep', NaN);
E = det.eyes;
searching = any(strcmp(T.status, {'search', 'lost'}));
pxPerM = P.cam.f_px/T.z;
% gate: eyes reopen where they closed, so it only widens after the blink window
gate = @(e) pxPerM*(P.trk.maxJumpM + P.trk.vHeadMax*min(0.5, max(0, t - T.tSeen(e) - P.trk.blinkMax)));

if numel(E) == 2 && det.pair
    % pair: E(1) is image-right by construction
    ok = true;
    if ~searching
        for e = 1:2
            if ~isnan(pred(1, e)) && hypot(E(e).uv(1) - pred(1, e), E(e).uv(2) - pred(2, e)) > 3*gate(e)   % a correctly spaced pair is strong evidence
                ok = false;
            end
        end
    end
    if ok
        for e = 1:2, obs.uv(:, e) = E(e).uv'; obs.eye{e} = E(e); obs.seen(e) = true; end
        obs.pairSeen = true;
    end
elseif ~isempty(E) && ~searching
    % single eye(s): assign each to the closest prediction
    for k = 1:numel(E)
        dmin = inf; best = 0; gBest = 0;
        for e = 1:2
            pe = pred(:, e);
            if isnan(pe(1)) && ~isnan(T.offset(1)) && ~isnan(pred(1, 3-e))
                pe = pred(:, 3-e) + (2*(e == 1) - 1)*T.offset;
            end
            dd = hypot(E(k).uv(1) - pe(1), E(k).uv(2) - pe(2));
            if dd < dmin, dmin = dd; best = e; gBest = gate(e); end
        end
        if best > 0 && dmin < gBest && ~obs.seen(best)
            obs.uv(:, best) = E(k).uv'; obs.eye{best} = E(k); obs.seen(best) = true;
        end
    end
end

% ---- state update
for e = 1:2
    if obs.seen(e)
        if T.tSeen(e) > -inf && t - T.tSeen(e) < 0.2 && ~isnan(T.uv(1, e))
            vNew = (obs.uv(:, e) - T.uv(:, e))/max(t - T.tSeen(e), 1e-3);
            T.vel(:, e) = 0.6*T.vel(:, e) + 0.4*vNew;
        else
            T.vel(:, e) = [0; 0];
        end
        T.uv(:, e) = obs.uv(:, e);
        T.tSeen(e) = t;
    elseif ~isnan(T.uv(1, e))
        T.uv(:, e) = pred(:, e);                    % coast
    end
end
if obs.pairSeen
    T.offset = obs.uv(:, 1) - obs.uv(:, 2);
    obs.sep = norm(T.offset);
    obs.mid = mean(obs.uv, 2);
elseif any(obs.seen) && ~isnan(T.offset(1))
    e = find(obs.seen, 1);
    obs.mid = obs.uv(:, e) - (2*(e == 1) - 1)*T.offset/2;
end
tMiss = t - max(T.tSeen);
if obs.pairSeen && searching
    T.status = 'track'; T.nLocks = T.nLocks + 1;
elseif any(obs.seen)
    if ~searching, T.status = 'track'; end
elseif ~searching
    if tMiss <= P.trk.blinkMax,      T.status = 'blink';
    elseif tMiss <= P.trk.lostAfter, T.status = 'blink';   % still searching around the last place
    else,                            T.status = 'lost'; T.uv(:) = NaN; T.vel(:) = 0;
    end
end
T.tLast = t;
obs.status = T.status;
end
