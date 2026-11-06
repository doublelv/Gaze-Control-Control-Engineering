function plot_paper_logs(sysidFile, trackFile, stepFile)
%PLOT_PAPER_LOGS  Figures from REAL rig logs for the paper / poster.
%   plot_paper_logs('artifacts/sysid_....mat', 'artifacts/logs/rig_track_camera_....mat')
%   plot_paper_logs(..., 'artifacts/logs/rig_step_....mat')   % after run_rig TASK = 'step'
%   1. open-loop 3 V step (sysid): measured angle vs fitted first-order model
%   2. face tracking: head, reference, carriage; real vs model replay (compare_log_sim)
%   3. loop period histogram
%   4. (optional) real step response vs model (sim_carriage_step with the logged loop periods)
%   Output: artifacts/paper/*.png
root = setup_paths();
outDir = fullfile(root, 'artifacts', 'paper'); if ~exist(outDir, 'dir'), mkdir(outDir); end

S = load(sysidFile);
f = figure('Color', 'w'); t = S.Ls.t(:); y = S.Ls.y(:); ok = ~isnan(y) & t < 0.5;
plot(t(ok), y(ok) - y(find(ok, 1)), 'o', 'MarkerSize', 3); hold on; grid on;
plot(t(ok), S.fit.thetaModel, 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('\theta output shaft [rad]');
legend('measured (3 V step)', sprintf('model: \\tau_m = %.1f ms, \\omega_\\infty = %.2f rad/s', S.fit.tau*1e3, S.fit.wInf), ...
       'Location', 'northwest');
title('Open-loop identification'); print(f, fullfile(outDir, 'fig_sysid.png'), '-dpng', '-r200');
fprintf('sysid: tau_m = %.1f ms, K_m = %.2f rad/(V s) at the output shaft, fit RMS %.4f rad\n', ...
        S.fit.tau*1e3, S.fit.wInf/3, S.fit.rmsErr);

R = compare_log_sim(trackFile);
f = figure('Color', 'w');
subplot(2,1,1); hv = ~isnan(R.head);
plot(R.t(hv), R.head(hv)*1e3, '.', 'MarkerSize', 3); hold on; grid on;
plot(R.t, R.ref*1e3, 'k--', R.t, R.yReal*1e3, R.t, R.ySim*1e3, 'LineWidth', 1);
ylabel('x [mm]'); legend('head (face)', 'reference', 'carriage (real)', 'carriage (model)');
title(sprintf('Face tracking: real vs model, RMS difference %.1f mm', R.rmsDiff*1e3));
subplot(2,1,2); plot(R.t, R.VReal, R.t, R.VSim, 'LineWidth', 1); grid on;
xlabel('t [s]'); ylabel('motor [V]'); legend('real', 'model');
print(f, fullfile(outDir, 'fig_sim_vs_real.png'), '-dpng', '-r200');

f = figure('Color', 'w'); lm = R.loopMs(R.loopMs > 0);
edges = 2.5:5:197.5; cnt = hist(min(lm, 197.5), edges); bar(edges, cnt, 1); grid on; xlabel('loop period [ms]'); ylabel('count');
title(sprintf('Loop period: median %.0f ms, mean %.0f ms', median(lm), mean(lm)));
print(f, fullfile(outDir, 'fig_loop_hist.png'), '-dpng', '-r200');

if nargin >= 3 && ~isempty(stepFile)
    T = load(stepFile); L = T.L; P = T.P; C = T.C; P.bSP = P.trk.bSP;
    M = build_axis_model(P.ax(1), P);
    tr = L.tReal; ok = ~isnan(L.y);
    k = find(ok & tr >= P.stepTest.period & tr < 2*P.stepTest.period);   % first up-step
    y0 = L.ref(1, k(1) - 1);
    tt = tr(k) - tr(k(1)); yy = (L.y(k) - y0)*M.r;
    [ts, xs] = sim_carriage_step(P, M, C, P.stepTest.size, diff(tr(k)), tt(end));
    sm = step_metrics(tt, yy, 0, P.stepTest.size); smS = step_metrics(ts, xs, 0, P.stepTest.size);
    f = figure('Color', 'w');
    plot(tt, yy*1e3, ts, xs*1e3, '--', 'LineWidth', 1.2); grid on; hold on;
    plot([0 tt(end)], P.stepTest.size*1e3*[1 1], 'k:');
    xlabel('t [s]'); ylabel('carriage x [mm]');
    legend(sprintf('real: OS %.1f %%', sm.OS), sprintf('model: OS %.1f %%', smS.OS), 'Location', 'southeast');
    title(sprintf('%.0f mm step: real vs model', P.stepTest.size*1e3));
    print(f, fullfile(outDir, 'fig_step_real.png'), '-dpng', '-r200');
end
end
