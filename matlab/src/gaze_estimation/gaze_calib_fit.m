function M = gaze_calib_fit(V, X, P)
%GAZE_CALIB_FIT  Fit PCCR -> screen mapping per eye from calibration data.
%   V : 2 x 2 x K per-target median features [component, eye, target] (NaN = missing)
%   X : 2 x K target positions on the screen plane [m] (x right, y up, world frame)
%   M.coef{e} : terms x 2 coefficients for eye e ([] if too few points)
%   M.order, M.rmsM (1x2 fit residual [m]), M.loo (1x2 leave-one-out error [deg]),
%   M.zCal (mean eye distance during calibration), M.created (date string)
%   Uses P.gcal.order (falls back to 1 when fewer than 8 points per eye).
K = size(X, 2);
M.order = P.gcal.order; M.coef = {[], []}; M.rmsM = [NaN NaN]; M.loo = [NaN NaN];
M.nPts = [0 0]; M.created = datestr(now, 'yyyy-mm-dd HH:MM');
for e = 1:2
    v = squeeze(V(:, e, :));
    if K == 1, v = v(:); end
    ok = all(~isnan(v), 1);
    M.nPts(e) = nnz(ok);
    ord = P.gcal.order;
    if nnz(ok) < 8, ord = 1; end
    if nnz(ok) < 4, continue, end
    A = gaze_poly_terms(v(:, ok), ord);
    C = A \ X(1:2, ok)';
    M.coef{e} = C; M.orderE(e) = ord;
    res = A*C - X(1:2, ok)';
    M.rmsM(e) = sqrt(mean(sum(res.^2, 2)));
    % leave-one-out angular error (honest accuracy estimate)
    idx = find(ok); err = zeros(1, numel(idx));
    for k = 1:numel(idx)
        tr = idx; tr(k) = [];
        Ck = gaze_poly_terms(v(:, tr), ord) \ X(1:2, tr)';
        pk = gaze_poly_terms(v(:, idx(k)), ord)*Ck;
        err(k) = norm(pk' - X(1:2, idx(k)));
    end
    M.loo(e) = atan(mean(err)/P.head.zDefault)*180/pi;
end
end
