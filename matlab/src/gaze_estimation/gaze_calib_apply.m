function G = gaze_calib_apply(M, F, P)
%GAZE_CALIB_APPLY  PCCR features -> gaze point on the screen.
%   G.valid : true if at least one calibrated eye had a feature
%   G.X     : 2x1 gaze point on the screen plane [m] (world x, y)
%   G.uv    : 2x1 normalised screen coordinates (0..1, origin top-left)
%   G.px    : 2x1 screen pixels,  G.eyes : 2x2 per-eye estimates [m]
G = struct('valid', false, 'X', [NaN; NaN], 'uv', [NaN; NaN], 'px', [NaN; NaN], 'eyes', nan(2, 2));
for e = 1:2
    if ~F.valid(e) || isempty(M.coef{e}), continue, end
    G.eyes(:, e) = (gaze_poly_terms(F.v(:, e), M.orderE(e))*M.coef{e})';
end
ok = ~isnan(G.eyes(1, :));
if ~any(ok), return, end
G.X = mean(G.eyes(:, ok), 2);                       % binocular average
G.uv = [ (G.X(1) - P.screen.centre(1))/P.screen.size(1) + 0.5;
        -(G.X(2) - P.screen.centre(2))/P.screen.size(2) + 0.5];
G.px = G.uv.*P.screen.res(:);
G.valid = true;
end
