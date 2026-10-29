function prior = eye_tracker_prior(T, t, P)
%EYE_TRACKER_PRIOR  Search windows for eye_detect at time t ([] = full frame).
if any(strcmp(T.status, {'search', 'lost'})) || all(isnan(T.uv(:)))
    prior = []; return
end
dt = min(t - T.tLast, 0.2);
uv = T.uv + T.vel*dt;
% an eye not seen for a while: place it from the other eye + last pair offset
for e = 1:2
    o = 3 - e;
    if isnan(uv(1, e)) && ~isnan(uv(1, o)) && ~isnan(T.offset(1))
        uv(:, e) = uv(:, o) + (2*(e == 1) - 1)*T.offset;
    end
end
grow = 1 + 2*min(1, max(0, t - max(T.tSeen))/P.trk.blinkMax);   % widen after misses
prior.uv = uv;
prior.z = T.z;
prior.half = P.det.roiHalf*grow;
end
