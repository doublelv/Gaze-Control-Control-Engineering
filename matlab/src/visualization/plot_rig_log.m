function plot_rig_log(L, P)
%PLOT_RIG_LOG  Diagnostic plots of a run_rig log (struct L or .mat file name).
%   Carriage reference / encoder, motor voltage, loop timing, link errors and,
%   for camera tracking, head position and tracker status.
if ischar(L) || isstring(L), s = load(L); L = s.L; P = s.P; end
t = L.t;
figure('Name', 'run_rig log', 'Position', [80 80 1000 800]);
subplot(4,1,1);
plot(t, L.ref(1,:)*P.belt.r*1e3, 'k--', t, L.y*P.belt.r*1e3, 'LineWidth', 1.2); hold on;
if isfield(L, 'head') && any(~isnan(L.head(1,:)))
    ok = ~isnan(L.head(1,:)); plot(t(ok), L.head(1,ok)*1e3, '-', 'Color', [0.2 0.7 0.3], 'LineWidth', 1.0);
end
grid on; ylabel('x [mm]'); legend('reference', 'carriage', 'head x', 'Location', 'best');
title(sprintf('%s (%s)', L.task, L.mode), 'Interpreter', 'none');
subplot(4,1,2); plot(t, L.V, 'LineWidth', 1.1); grid on; ylabel('motor [V]');
subplot(4,1,3); plot(t, L.loopMs, '.'); grid on; ylabel('loop [ms]');
hold on; plot(t([1 end]), [1 1]*P.Ts*1e3, 'r--');
subplot(4,1,4);
if isfield(L, 'statusCode')
    ok = ~isnan(L.statusCode); stairs(t(ok), L.statusCode(ok), 'LineWidth', 1.1); set(gca, 'YTick', 0:3, 'YTickLabel', {'search', 'track', 'blink', 'lost'});
    ylim([-0.5 3.5]);
end
grid on; xlabel('t [s]'); ylabel('tracker');
end
