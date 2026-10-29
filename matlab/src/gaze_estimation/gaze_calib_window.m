function ui = gaze_calib_window(P, cmd, arg)
%GAZE_CALIB_WINDOW  Full-screen black window with a calibration dot.
%   ui = gaze_calib_window(P, 'open')        open on monitor P.screen.monitor
%   gaze_calib_window(ui, 'target', uv)      animate the dot to uv (0..1, top-left)
%   gaze_calib_window(ui, 'done')            close
switch cmd
    case 'open'
        mp = get(0, 'MonitorPositions');
        m = mp(min(P.screen.monitor, size(mp, 1)), :);
        ui.fig = figure('Color', 'k', 'MenuBar', 'none', 'ToolBar', 'none', 'NumberTitle', 'off', ...
                        'Name', 'Gaze calibration - look at the dot', 'Position', m);
        try, ui.fig.WindowState = 'fullscreen'; catch, end
        ui.ax = axes('Parent', ui.fig, 'Position', [0 0 1 1], 'Color', 'k', 'XLim', [0 1], 'YLim', [0 1], ...
                     'YDir', 'reverse', 'Visible', 'off');
        hold(ui.ax, 'on');
        ui.dot = plot(ui.ax, 0.5, 0.5, 'o', 'MarkerSize', 30, 'MarkerFaceColor', [1 0.3 0.2], 'MarkerEdgeColor', 'w');
        ui.ctr = plot(ui.ax, 0.5, 0.5, '.', 'MarkerSize', 8, 'Color', 'k');
        ui.txt = text(ui.ax, 0.5, 0.5, 'Keep your head still and look at the dot', 'Color', 'w', ...
                      'HorizontalAlignment', 'center', 'FontSize', 18);
        ui.show = true;
        drawnow; pause(1.5); set(ui.txt, 'String', '');
    case 'target'
        ui = P; uv = arg;
        for s = linspace(30, 12, 12)                 % shrink to draw attention
            set(ui.dot, 'XData', uv(1), 'YData', uv(2), 'MarkerSize', s);
            set(ui.ctr, 'XData', uv(1), 'YData', uv(2));
            drawnow; pause(0.02);
        end
    case 'done'
        ui = P;
        if isfield(ui, 'fig') && viz_alive(ui.fig), close(ui.fig); end
end
end
