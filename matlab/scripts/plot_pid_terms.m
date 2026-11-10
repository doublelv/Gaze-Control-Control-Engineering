function plot_pid_terms(logFile, tWin)
%PLOT_PID_TERMS  Show the P, I and D terms of the carriage PID in a run_rig log.
%   plot_pid_terms('artifacts/logs/rig_step_....mat')          % whole log
%   plot_pid_terms('artifacts/logs/rig_step_....mat', [3 9])   % only 3..9 s
%   Top: reference and carriage position [mm].
%   Bottom [V]: P term Kp*(r - y) (on the error), I term, D term (-Kd * filtered
%   speed, on the measurement) and the motor voltage.
%   Set-point weighting: the controller uses Kp*(b*r - y) = Kp*(r - y) - Kp*(1-b)*r.
%   The part -Kp*(1-b)*r depends only on the (absolute) position reference and is
%   cancelled by the integrator in steady state; it is shown inside the I curve
%   ("I + set-point weight"), so P shows the error and the three curves still add
%   up to the controller output.
%   Gaps = motor switched off in the hold band (the PID is not evaluated there).
%   Saves a PNG next to the log.
S = load(logFile); L = S.L; C = S.C; r = S.P.belt.r;
if ~isfield(L, 'uPID'), error('plot_pid_terms: this log has no uPID (recorded before 2026-11-10)'); end
t = L.tReal; ok = ~isnan(t) & ~isnan(L.y);
if nargin >= 2 && ~isempty(tWin), ok = ok & t >= tWin(1) & t <= tWin(2); end
t = t(ok);
f = figure('Name', 'PID terms', 'Position', [100 100 900 600]);
subplot(2, 1, 1);
plot(t, L.ref(1, ok)*r*1e3, 'k--', t, L.y(ok)*r*1e3, 'b-', 'LineWidth', 1.2);
ylabel('position [mm]'); legend('reference', 'carriage', 'Location', 'best'); grid on;
title(sprintf('%s: K_p %.2f V/rad, K_i %.2f V/(rad s), K_d %.3f V s/rad', ...
      strrep(L.task, '_', '\_'), C.Kp, C.Ki, C.Kd));
subplot(2, 1, 2);
u = L.uPID(:, ok);
b = S.P.trk.bSP; sw = C.Kp*(1 - b)*L.ref(1, ok);   % set-point-weight part
u(1, :) = u(1, :) + sw;                          % P on the error: Kp*(r - y)
u(2, :) = u(2, :) - sw;                          % I + set-point weight
plot(t, u(1, :), 'r-', t, u(2, :), 'g-', t, u(3, :), 'm-', t, L.V(ok), 'k:', 'LineWidth', 1.2);
ylabel('voltage [V]'); xlabel('time [s]'); grid on;
legend('P term K_p(r-y)', 'I term (+ set-point weight)', 'D term', 'motor voltage', 'Location', 'best');
[d, n] = fileparts(logFile);
png = fullfile(d, [n '_pid_terms.png']);
print(f, png, '-dpng', '-r150');
fprintf('P term: |max| %.2f V, I term: |max| %.2f V, D term: |max| %.2f V\n', ...
        max(abs(u(1, :))), max(abs(u(2, :))), max(abs(u(3, :))));
fprintf('Figure: %s\n', png);
end
