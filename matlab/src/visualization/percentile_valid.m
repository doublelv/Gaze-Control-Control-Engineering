function q = percentile_valid(x, p)
%PERCENTILE_VALID  p-th percentile (0..100) of the non-NaN values, linear
%   interpolation, no toolbox needed (prctile is in the Statistics Toolbox).
x = sort(x(~isnan(x(:))));
if isempty(x), q = NaN; return, end
r = 1 + (numel(x) - 1)*p/100;
lo = floor(r); hi = ceil(r);
q = x(lo) + (r - lo)*(x(hi) - x(lo));
end
