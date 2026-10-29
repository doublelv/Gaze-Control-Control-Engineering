function h = viz_detection(ax, img, det, obs, h)
%VIZ_DETECTION  Camera image with detection overlays (fast update on reuse).
%   First call: h = viz_detection(ax, img, det, obs) creates the graphics.
%   Next calls: h = viz_detection(ax, img, det, obs, h) only updates data.
%   Overlays: search regions (yellow), candidates (grey +), pupils (green circle =
%   right eye, cyan = left eye), glints (red x), status text.
%   Units: image pixels, v down (MATLAB image convention).
if nargin < 4, obs = []; end
if nargin < 5 || isempty(h)
    cla(ax);
    h.img = image(ax, repmat(img, 1, 1, 3)); axis(ax, 'image'); hold(ax, 'on');
    set(ax, 'XTick', [], 'YTick', []);
    h.roi = plot(ax, NaN, NaN, '-', 'Color', [1 0.85 0.2]);
    h.cand = plot(ax, NaN, NaN, '+', 'Color', [0.7 0.7 0.7], 'MarkerSize', 8);
    h.pup(1) = plot(ax, NaN, NaN, 'o', 'Color', [0.2 1 0.3], 'LineWidth', 1.5, 'MarkerSize', 14);
    h.pup(2) = plot(ax, NaN, NaN, 'o', 'Color', [0.2 0.9 1], 'LineWidth', 1.5, 'MarkerSize', 14);
    h.gl = plot(ax, NaN, NaN, 'x', 'Color', [1 0.25 0.2], 'MarkerSize', 8, 'LineWidth', 1.2);
    h.txt = text(ax, 10, 25, '', 'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold', ...
                 'BackgroundColor', [0 0 0], 'Interpreter', 'none');
else
    set(h.img, 'CData', repmat(img, 1, 1, 3));
end
% search regions as closed rectangles separated by NaN
R = det.rois; xs = []; ys = [];
for k = 1:size(R, 1)
    xs = [xs, R(k,3)-1, R(k,4), R(k,4), R(k,3)-1, R(k,3)-1, NaN]; %#ok<AGROW>
    ys = [ys, R(k,1)-1, R(k,1)-1, R(k,2), R(k,2), R(k,1)-1, NaN];   %#ok<AGROW>
end
set(h.roi, 'XData', xs, 'YData', ys);
if ~isempty(det.cand), set(h.cand, 'XData', det.cand(:,1) + 0.5, 'YData', det.cand(:,2) + 0.5);
else, set(h.cand, 'XData', NaN, 'YData', NaN); end
gx = []; gy = [];
for e = 1:2
    uv = [NaN NaN];
    if ~isempty(obs) && obs.seen(e)
        uv = obs.uv(:, e)'; g = obs.eye{e}.glints;
    elseif isempty(obs) && numel(det.eyes) >= e
        uv = det.eyes(e).uv; g = det.eyes(e).glints;
    else
        g = zeros(0, 2);
    end
    set(h.pup(e), 'XData', uv(1) + 0.5, 'YData', uv(2) + 0.5);   % +0.5: pixel-centre -> image() coords
    gx = [gx; g(:,1) + 0.5]; gy = [gy; g(:,2) + 0.5]; %#ok<AGROW>
end
set(h.gl, 'XData', gx, 'YData', gy);
if ~isempty(obs), status = obs.status; else, status = sprintf('%d eye(s)', numel(det.eyes)); end
set(h.txt, 'String', sprintf('%s | candidates %d', status, det.nCand));
end
