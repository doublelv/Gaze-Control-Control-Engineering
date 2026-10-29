function V = viz_live_update(V, t, img, det, obs, H, G, xCar, info)
%VIZ_LIVE_UPDATE  Update the live view (rate-limited to P.viz.rate).
%   t: time [s], img: camera frame, det/obs/H/G: detector, tracker, head and
%   gaze outputs (G may be []), xCar: carriage [m] (NaN = no rig), info: struct
%   with extra status lines (field name -> value), shown in the text panel.
P = V.P;
V.hist(:, end+1) = [t; H.pos(1)*1e3; xCar*1e3; (H.z - 0.6)*1e3];
V.hist = V.hist(:, V.hist(1,:) > t - 10);
if t - V.lastDraw < 1/P.viz.rate || ~viz_alive(V.fig), return, end
V.lastDraw = t;
V.hDet = viz_detection(V.axCam, img, det, obs, V.hDet);
for e = 1:2                                           % eye close-ups
    ax = V.axEye(e);
    if obs.seen(e)
        c = round(obs.uv(:, e)); r = 20;
        r1 = max(1, c(2) - r); r2 = min(size(img,1), c(2) + r);
        c1 = max(1, c(1) - r); c2 = min(size(img,2), c(1) + r);
        crop = img(r1:r2, c1:c2);
        if isempty(V.hEye{e})
            V.hEye{e} = image(ax, repmat(crop, 1, 1, 3)); axis(ax, 'image'); set(ax, 'XTick', [], 'YTick', []);
            hold(ax, 'on'); V.hEyeP(e) = plot(ax, NaN, NaN, 'g+', 'MarkerSize', 12);
            title(ax, sprintf('%s eye', pick(e == 1, 'right', 'left')), 'Color', 'w');
        else
            set(V.hEye{e}, 'CData', repmat(crop, 1, 1, 3));
            set(ax, 'XLim', [0.5 size(crop,2)+0.5], 'YLim', [0.5 size(crop,1)+0.5]);
        end
        set(V.hEyeP(e), 'XData', obs.uv(1,e) - c1 + 1.5, 'YData', obs.uv(2,e) - r1 + 1.5);
    end
end
h = V.hist;
set(V.hHead, 'XData', h(1,:), 'YData', h(2,:));
set(V.hCar,  'XData', h(1,:), 'YData', h(3,:));
set(V.hZ,    'XData', h(1,:), 'YData', h(4,:));
if size(h, 2) > 1, set(V.axHist, 'XLim', [max(h(1,1), t - 10) max(t, h(1,1) + 1)]); end
if ~isempty(G) && G.valid
    V.trail = [V.trail, G.uv]; V.trail = V.trail(:, max(1, end-30):end);
    set(V.gaze, 'XData', G.uv(1), 'YData', G.uv(2));
    set(V.gazeTrail, 'XData', V.trail(1,:), 'YData', V.trail(2,:));
end
lines = {sprintf('t        %7.2f s', t), sprintf('status   %s', obs.status), ...
         sprintf('head x   %7.1f mm', H.pos(1)*1e3), sprintf('head y   %7.1f mm', H.pos(2)*1e3), ...
         sprintf('dist z   %7.1f mm', H.z*1e3), sprintf('roll     %7.1f deg', H.roll*180/pi)};
if ~isnan(xCar), lines{end+1} = sprintf('carriage %7.1f mm', xCar*1e3); end
if ~isempty(G) && G.valid, lines{end+1} = sprintf('gaze     %4.0f, %4.0f px', G.px); end
if nargin >= 9 && ~isempty(info)
    fn = fieldnames(info);
    for k = 1:numel(fn), lines{end+1} = sprintf('%-8s %s', fn{k}, num2str(info.(fn{k}))); end %#ok<AGROW>
end
set(V.txt, 'String', strjoin(lines, newline));
viz_drawnow();
end
