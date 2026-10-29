function V = viz_live_init(P, title_)
%VIZ_LIVE_INIT  Live view: camera + overlays, eye close-ups, gaze on the screen,
%   head / carriage history. Close the window to stop the running script.
V.fig = figure('Name', title_, 'NumberTitle', 'off', 'Color', [0.12 0.12 0.12], ...
               'Position', [60 60 1280 760]);
V.axCam = axes('Parent', V.fig, 'Position', [0.02 0.30 0.62 0.68]);
V.axEye(1) = axes('Parent', V.fig, 'Position', [0.02 0.03 0.15 0.24]);
V.axEye(2) = axes('Parent', V.fig, 'Position', [0.18 0.03 0.15 0.24]);
V.axScr = axes('Parent', V.fig, 'Position', [0.68 0.62 0.30 0.34], 'Color', 'k', ...
               'XLim', [0 1], 'YLim', [0 1], 'YDir', 'reverse', 'XColor', [0.6 0.6 0.6], 'YColor', [0.6 0.6 0.6]);
title(V.axScr, 'gaze on screen', 'Color', 'w'); hold(V.axScr, 'on'); box(V.axScr, 'on');
V.gaze = plot(V.axScr, NaN, NaN, 'o', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.4 0.2], 'MarkerEdgeColor', 'w');
V.gazeTrail = plot(V.axScr, NaN, NaN, '-', 'Color', [0.8 0.5 0.35]);
V.axHist = axes('Parent', V.fig, 'Position', [0.40 0.06 0.58 0.20], 'Color', [0.08 0.08 0.08], ...
                'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
hold(V.axHist, 'on'); grid(V.axHist, 'on');
V.hHead = plot(V.axHist, NaN, NaN, '-', 'Color', [0.3 0.8 1], 'LineWidth', 1.3);
V.hCar  = plot(V.axHist, NaN, NaN, '-', 'Color', [1 0.6 0.2], 'LineWidth', 1.3);
V.hZ    = plot(V.axHist, NaN, NaN, '--', 'Color', [0.6 1 0.6]);
ylabel(V.axHist, '[mm]'); legend(V.axHist, {'head x', 'carriage x', 'distance z - 600'}, ...
       'TextColor', 'w', 'Location', 'northwest', 'Color', [0.15 0.15 0.15]);
V.axTxt = axes('Parent', V.fig, 'Position', [0.68 0.30 0.30 0.30], 'Visible', 'off');
V.txt = text(V.axTxt, 0, 1, '', 'Color', 'w', 'FontName', 'Consolas', 'FontSize', 10, ...
             'VerticalAlignment', 'top', 'Interpreter', 'none');
V.hDet = []; V.hEye = {[], []};
V.hist = zeros(4, 0);                 % [t; headX; carX; z]
V.trail = zeros(2, 0);
V.lastDraw = -inf; V.P = P;
end
