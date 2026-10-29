function m = median_valid(x)
%MEDIAN_VALID  Median of the non-NaN elements (NaN if none). Works in MATLAB and Octave.
x = x(~isnan(x));
if isempty(x), m = NaN; else, m = median(x); end
end
