%COMPARE_P_PI_PID  Why PID: P-only, PI and PID on the carriage model (simulation).
%   Same Kp in all three (from design_pid). 50 mm step, loop period 46 ms (median
%   with the camera) on the nonlinear model (friction, saturation, PWM, encoder,
%   one-loop delay; sim_carriage_step). Plus the phase margin of PI and PID
%   against the loop period (1.5 loop periods of delay, loop_margins).
%   P-only : the Coulomb friction stops the carriage short -> steady-state error.
%   PI     : the integrator removes that error.
%   PID    : the D term adds phase lead -> more phase margin for the same Kp, Ki.
%   Saves artifacts/figures/compare_p_pi_pid.png and prints the numbers.
root = setup_paths();
P = params(); P.bSP = P.trk.bSP;
M = build_axis_model(P.ax(1), P);
C = design_pid(M, P.ax(1), P.Ts);
Cp = C; Cp.Ki = 0; Cp.Kd = 0;
Cpi = C; Cpi.Kd = 0;
names = {'P only', 'PI', 'PID'}; Cs = {Cp, Cpi, C}; col = {'r', 'b', 'k'};
Tl = 0.046; xs = 0.05;
f = figure('Name', 'P vs PI vs PID', 'Position', [100 100 1000 420]);
subplot(1, 2, 1); hold on; grid on;
for i = 1:3
    Pi = P; if i == 1, Pi.bSP = 1; end           % P only: no set-point weight (would leave 40 % error)
    [t, x] = sim_carriage_step(Pi, M, Cs{i}, xs, Tl, 3);
    plot(t, x*1e3, col{i}, 'LineWidth', 1.3);
    e = (xs - x(end))*1e3; os = max(x/xs - 1)*100;
    j = find(abs(x - xs) > 0.05*xs, 1, 'last'); ts = t(min(j + 1, end));
    if isempty(j), ts = 0; end
    fprintf('%-7s Kp %.2f Ki %.2f Kd %.3f: overshoot %.1f %%, settling (5 %%) %.2f s, final error %.2f mm\n', ...
            names{i}, Cs{i}.Kp, Cs{i}.Ki, Cs{i}.Kd, os, ts, e);
end
plot([0 3], [xs xs]*1e3, 'k:');
xlabel('time [s]'); ylabel('carriage [mm]'); legend(names, 'Location', 'southeast');
title(sprintf('50 mm step, model, %.0f ms loop', Tl*1e3));
subplot(1, 2, 2); hold on; grid on;
Ts = 0.02:0.002:0.12; pm = nan(2, numel(Ts));
for j = 1:numel(Ts)
    a = loop_margins(M, Cpi, Ts(j)); b = loop_margins(M, C, Ts(j)); pm(:, j) = [a.PM; b.PM];
end
plot(Ts*1e3, pm(1, :), 'b-', Ts*1e3, pm(2, :), 'k-', 'LineWidth', 1.3);
xlabel('loop period [ms]'); ylabel('phase margin [deg]'); legend('PI', 'PID', 'Location', 'northeast');
title('Phase margin vs loop period');
for T = [0.02 0.046 0.118]
    a = loop_margins(M, Cpi, T); b = loop_margins(M, C, T);
    fprintf('loop %3.0f ms: phase margin PI %.1f deg, PID %.1f deg; gain margin PI %.1f dB, PID %.1f dB\n', ...
            T*1e3, a.PM, b.PM, a.GM, b.GM);
end
d = fullfile(root, 'artifacts', 'figures'); if ~exist(d, 'dir'), mkdir(d); end
print(f, fullfile(d, 'compare_p_pi_pid.png'), '-dpng', '-r150');
fprintf('Figure: %s\n', fullfile(d, 'compare_p_pi_pid.png'));
