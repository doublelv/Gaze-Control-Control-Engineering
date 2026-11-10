function plot_pid_terms(logFile, tWin)
%PLOT_PID_TERMS  Show the P, I and D terms of the carriage PID in a run_rig log.
%   plot_pid_terms('artifacts/logs/rig_step_....mat')          % whole log
%   plot_pid_terms('artifacts/logs/rig_step_....mat', [3 9])   % only 3..9 s
%   Top: reference and carriage position [mm].
%   Bottom: P term Kp*(b*r - y), I term (integral of the error times Ki),
%   D term (-Kd * filtered speed, on the measurement) and the motor voltage [V].
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
plot(t, u(1, :), 'r-', t, u(2, :), 'g-', t, u(3, :), 'm-', t, L.V(ok), 'k:', 'LineWidth', 1.2);
ylabel('voltage [V]'); xlabel('time [s]'); grid on;
legend('P term', 'I term', 'D term', 'motor voltage', 'Location', 'best');
[d, n] = fileparts(logFile);
png = fullfile(d, [n '_pid_terms.png']);
print(f, png, '-dpng', '-r150');
fprintf('P term: |max| %.2f V, I term: |max| %.2f V, D term: |max| %.2f V\n', ...
        max(abs(u(1, :))), max(abs(u(2, :))), max(abs(u(3, :))));
fprintf('Figure: %s\n', png);
end
