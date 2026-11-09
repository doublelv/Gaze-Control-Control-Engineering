function det = eye_in_face(img, box, P)
%EYE_IN_FACE  Find both eyes (iris centres) inside a face box.
%   Used by run_rig track_camera when P.det.method = 'face'. At ~50 cm the
%   OV9281 sees each iris only ~10 px wide and the pupil is not darker than a
%   dark iris, so a pupil detector finds nothing; the iris (the darkest round
%   spot in each eye region) is still clear.
%   img : H x W uint8 frame,  box : [x y w h] from face_find ([] -> no eyes)
%   det : same fields as eye_detect (eyes(1) = image right), no glints.
%   Steps per eye: region (left/right half of the eye band of the box) ->
%   smoothing with an iris-sized window (thin lash lines and eyebrow hairs fade,
%   the iris stays dark) -> darkest point with skin above and below -> mean shift
%   -> open-eye test: iris clearly darker than the region (closed eye = only
%   a thin dark line -> fails).
D = P.det;
det = struct('eyes', struct('uv', {}, 'r', {}, 'conf', {}, 'glints', {}, 'gc', {}, 'pccr', {}), ...
             'pair', false, 'nCand', 0, 'cand', zeros(0, 5), 'rois', zeros(0, 4));
if isempty(box), return, end
[H, W] = size(img);
bw = box(3); bh = box(4);
r1 = max(1, round(box(2) + (P.face.eyeLevel + D.faceBand(1))*bh));
r2 = min(H, round(box(2) + (P.face.eyeLevel + D.faceBand(2))*bh));
rIris = D.faceIris*bw;                          % expected iris radius [px]
cols = [D.faceCols(1) 0.5; 0.5 D.faceCols(2)];
if r2 - r1 < 4, return, end
for s = 1:2
    c1 = max(1, round(box(1) + cols(s, 1)*bw));
    c2 = min(W, round(box(1) + cols(s, 2)*bw));
    if c2 - c1 < 4, continue, end
    det.rois(end+1, :) = [r1 r2 c1 c2];
    G = double(img(r1:r2, c1:c2));
    S = box_mean(G, 2*rIris);                   % iris-sized smoothing
    m = min(round(1.5*rIris), floor(size(S, 2)/4));   % hair at the box edge is not an eye
    S(:, [1:m, end-m+1:end]) = inf;
    ref = median(G(:));                         % skin level of the region
    % darkest spot that has brighter skin above AND below it (an iris; a strand
    % of hair or the shadow beside the face is dark from top to bottom)
    dv = max(2, round(D.faceEncl*rIris));
    found = false;
    for try_ = 1:4
        [sMin, i] = min(S(:));
        if ~isfinite(sMin), break, end
        [vi, ui] = ind2sub(size(S), i);
        lvl = sMin + 0.5*(ref - sMin);
        up = mean(G(max(1, vi - dv - 1):max(1, vi - dv + 1), ui));
        dn = mean(G(min(end, vi + dv - 1):min(end, vi + dv + 1), ui));
        if (vi - dv >= 1 && up > lvl) && (vi + dv <= size(G, 1) && dn > lvl)
            found = true; break
        end
        S(:, max(1, ui - m):min(end, ui + m)) = inf;   % not an eye: try the next spot
    end
    if ~found, continue, end
    contrast = (ref - sMin)/max(ref, 1);
    % centre: mean shift over the dark pixels in an iris-sized disc. Every dark
    % pixel counts the same, so the (darker) lash line at the disc edge does not
    % pull the centre up.
    [X, Y] = meshgrid(1:size(G, 2), 1:size(G, 1));
    dark = G < lvl;
    u = ui; v = vi; ok = true;
    for it = 1:4
        w = dark & hypot(X - u, Y - v) <= 1.2*rIris;
        if ~any(w(:)), ok = false; break, end
        u = mean(X(w)); v = mean(Y(w));
    end
    if ~ok, continue, end
    u = u - 0.5 + (c1 - 1); v = v - 0.5 + (r1 - 1);   % full-image px, pixel centre = c - 0.5
    conf = min(1, contrast/(2*D.faceMinContrast));
    det.cand(end+1, :) = [u v rIris conf contrast];   % (5th column: contrast, not area)
    if contrast < D.faceMinContrast, continue, end    % closed eye / no eye
    det.eyes(end+1) = struct('uv', [u v], 'r', rIris, 'conf', conf, ...
                             'glints', zeros(0, 2), 'gc', [NaN NaN], 'pccr', [NaN NaN]);
end
det.nCand = size(det.cand, 1);
if numel(det.eyes) == 2
    us = arrayfun(@(x) x.uv(1), det.eyes);
    [~, o] = sort(us, 'descend');
    det.eyes = det.eyes(o);
    d = det.eyes(1).uv - det.eyes(2).uv;
    roll = abs(atan2(d(2), d(1)))*180/pi;
    det.pair = norm(d) > D.faceSep(1)*bw && norm(d) < D.faceSep(2)*bw && roll < D.maxRollDeg;
end
end
