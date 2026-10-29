function m = step_metrics(t, y, y0, yf)
%STEP_METRICS  Rise time (10-90%), overshoot, 2% settling time, steady-state error.
dy = yf - y0;
yn = (y - y0)/dy;
i10 = find(yn >= 0.1, 1); i90 = find(yn >= 0.9, 1);
if isempty(i10) || isempty(i90), m.tr = NaN; else, m.tr = t(i90) - t(i10); end
m.OS = max(0, (max(yn) - 1)*100);
iout = find(abs(yn - 1) > 0.02, 1, 'last');
if isempty(iout), m.ts = 0;
elseif iout == numel(t), m.ts = NaN;           % never settled
else, m.ts = t(iout+1) - t(1);
end
nTail = max(1, round(0.1*numel(y)));
m.ess = yf - mean(y(end-nTail+1:end));
end
