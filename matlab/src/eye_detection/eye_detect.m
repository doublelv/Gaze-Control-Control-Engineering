function det = eye_detect(img, P, prior)
%EYE_DETECT  Dark-pupil + glint detector for the OV9281 IR image.
%   img   : H x W mono uint8 (full camera frame)
%   prior : [] for a full-frame search, or a struct from eye_tracker_prior:
%           .uv (2x2 predicted [right left] pupil positions, full px, NaN = unknown)
%           .z  (expected eye distance [m]), .half (1x2 window half-size [px])
%   det.eyes : 1xK struct (K = 0..2), sorted by u descending (eyes(1) = image right):
%       .uv     pupil centre [u v], full-image px (pixel centres at c-0.5, r-0.5), v down
%       .r      pupil radius [px],  .conf  confidence 0..1
%       .glints Nx2 glint centres [px],  .gc  mean glint position (NaN if none)
%       .pccr   pupil - glint vector [px] (NaN if no glint)
%   det.pair  : true if the two eyes form a plausible pair (spacing, roll, size)
%   det.nCand : number of dark candidates, det.cand: [u v r conf area] per candidate
%   det.rois  : regions searched [r1 r2 c1 c2]
%   Steps: region -> local-contrast dark mask -> components -> shape/size/contrast
%   score -> best eye pair -> full-resolution sub-pixel refinement (glints inside
%   the pupil are filled) -> glint search around each pupil.
D = P.det; f = P.cam.f_px;
G0 = double(img);
[H, W] = size(G0);
det = struct('eyes', struct('uv', {}, 'r', {}, 'conf', {}, 'glints', {}, 'gc', {}, 'pccr', {}), ...
             'pair', false, 'nCand', 0, 'cand', zeros(0, 5), 'rois', zeros(0, 4));

% ---------- 1. search regions
if nargin < 3 || isempty(prior) || all(isnan(prior.uv(:)))
    rois = [1 H 1 W]; scale = D.down; zr = D.zRange;
else
    rois = zeros(0, 4);
    for e = 1:2
        if any(isnan(prior.uv(:, e))), continue, end
        u = prior.uv(1, e); v = prior.uv(2, e);
        rois(end+1, :) = [max(1, floor(v - prior.half(2))) min(H, ceil(v + prior.half(2))) ...
                          max(1, floor(u - prior.half(1))) min(W, ceil(u + prior.half(1)))]; %#ok<AGROW>
    end
    rois = merge_rois(rois);
    scale = 1;
    zr = prior.z*[1 - D.ipdTol, 1 + D.ipdTol];
end
det.rois = rois;
rMinPx = f*D.rPupilMm(1)*1e-3/zr(2);
rMaxPx = f*D.rPupilMm(2)*1e-3/zr(1);

% ---------- 2-4. candidates in every region
cand = zeros(0, 5);                         % [u v r conf area] in full-image px
for k = 1:size(rois, 1)
    R = rois(k, :);
    G = G0(R(1):R(2), R(3):R(4));
    if scale > 1, G = min_down(G, scale); end
    B = box_mean(G, D.bgWin/scale);
    mask = (G < D.absDark) & (B - G > D.darkK*B) & (B > D.minBg);
    if nnz(mask) > 0.05*numel(mask)          % too much dark (bad exposure): keep darkest part
        v = sort(G(mask)); mask = mask & (G <= v(round(0.05*numel(mask))));
    end
    [Lb, n] = cc_label(mask);
    if n == 0, continue, end
    aMin = max(1, 0.5*pi*(rMinPx/scale)^2);
    aMax = 2.0*pi*(rMaxPx/scale)^2 + 2;
    for j = 1:n
        [rr, cc] = find(Lb == j);
        a = numel(rr);
        if a < aMin || a > aMax, continue, end
        ii = sub2ind(size(G), rr, cc);
        w = B(ii) - G(ii);                   % darkness weight
        mu = [sum(w.*cc), sum(w.*rr)]/sum(w);
        if a >= 6                            % shape is not measurable on tinier blobs
            Cv = cov([cc rr]); ev = sort(eig(Cv + eye(2)/12));
            circ = sqrt(ev(1)/ev(2));
        else
            circ = 1;
        end
        if circ < D.minCirc, continue, end
        contrast = mean(w)/max(mean(B(ii)), 1);
        gs = sort(G(ii)); gDark = mean(gs(1:max(1, ceil(a/3))));      % darkest third: not affected by a glint
        if gDark > (1 - D.darkBlobK)*mean(B(ii)), continue, end        % not dark enough (eyebrow, shadow)
        % enclosure: a pupil is surrounded on ALL sides by brighter iris/sclera
        ra = sqrt(a/pi); gIn = mean(G(ii)); lvl = gIn + 0.4*(mean(B(ii)) - gIn);
        th = (0:15)*pi/8; encl = 0;
        for rk = [ra + 1.5, ra + 2.5]
            uu = round(mu(1) + rk*cos(th)); vv = round(mu(2) + rk*sin(th));
            okp = uu >= 1 & uu <= size(G, 2) & vv >= 1 & vv <= size(G, 1);
            vals = -inf(size(th)); vals(okp) = G(sub2ind(size(G), vv(okp), uu(okp)));
            encl = encl + mean(vals > lvl)/2;
        end
        if encl < D.minEncl, continue, end
        rPx = sqrt(a/pi)*scale;
        sizeOk = exp(-max(0, log(max(rPx, 0.5)/rMaxPx))^2*4 - max(0, log(rMinPx/max(rPx, 0.5)))^2*4);
        conf = min(1, contrast*1.3) * min(1, circ/0.8) * sizeOk * encl;
        if conf < D.minConf, continue, end
        u = (mu(1) - 0.5)*scale + (R(3) - 1);  % region px -> full-image px (pixel centre = c - 0.5)
        v = (mu(2) - 0.5)*scale + (R(1) - 1);
        cand(end+1, :) = [u v rPx conf a]; %#ok<AGROW>
    end
end
det.nCand = size(cand, 1);
det.cand = cand;
if isempty(cand), return, end

% ---------- 5. best pair (or best single)
best = []; bestS = 0;
for i = 1:size(cand, 1)
    for j = i+1:size(cand, 1)
        du = cand(i,1) - cand(j,1); dv = cand(i,2) - cand(j,2);
        sep = hypot(du, dv);
        z = f*D.ipd/max(sep, 1);
        roll = abs(atan2(dv, du)); roll = min(roll, pi - roll)*180/pi;
        if z < zr(1)*(1 - D.ipdTol) || z > zr(2)*(1 + D.ipdTol) || roll > D.maxRollDeg, continue, end
        rr = max(cand(i,3), cand(j,3))/max(min(cand(i,3), cand(j,3)), 0.3);
        if rr > 2.5, continue, end
        rExp = f*2.5e-3/z;                   % typical 2.5 mm pupil radius at that distance
        sz = exp(-log(mean(cand([i j],3))/rExp)^2);
        s = cand(i,4)*cand(j,4)*sz;
        if s > bestS, bestS = s; best = [i j]; end
    end
end
if ~isempty(best)
    sel = best; det.pair = true;
else
    [~, sel] = max(cand(:, 4));
end

% ---------- 6-7. refine at full resolution + glints
for k = 1:numel(sel)
    e = refine(G0, cand(sel(k), :), D);
    if ~isempty(e), det.eyes(end+1) = e; end
end
if numel(det.eyes) == 2
    us = arrayfun(@(x) x.uv(1), det.eyes);
    [~, o] = sort(us, 'descend');
    det.eyes = det.eyes(o);
else
    det.pair = false;
end
end

% =====================================================================
function e = refine(G0, c, D)
% Sub-pixel pupil centre + radius at full resolution, then glints.
e = [];
[H, W] = size(G0);
rad = max(6, ceil(4*c(3)));
uc = c(1); vc = c(2);
r1 = max(1, floor(vc - rad)); r2 = min(H, ceil(vc + rad));
c1 = max(1, floor(uc - rad)); c2 = min(W, ceil(uc + rad));
G = G0(r1:r2, c1:c2);
[nr, nc] = size(G);
[X, Y] = meshgrid((c1:c2) - 0.5, (r1:r2) - 0.5);
dist = hypot(X - uc, Y - vc);
ring = G(dist > c(3) + 1.0 & dist < c(3) + 2.5);    % iris just outside the pupil
if isempty(ring), ring = G(:); end
bg = median(ring);                               % iris level around the pupil
core = G(dist <= max(1.0, 0.6*c(3)));
if isempty(core), core = G(:); end
lo = min(core);                                  % pupil level
if bg - lo < 15, return, end                     % no contrast
gmax = max(G(:));
% glint zone (glint core + 2 px). It is excluded SYMMETRICALLY: a pixel is
% used only if its mirror image about the current centre is outside the zone
% too, so hiding part of the pupil cannot pull the centre toward the glint.
gz = small_bright(G, G >= max(D.glintMin, D.glintThr*gmax) & dist <= c(3) + 3, D.maxGlintArea);
gz = conv2(double(gz), ones(3), 'same') > 0;     % + 1 px halo
cv_ = min(1, max(0, (bg - G)/(bg - lo)));        % dark coverage 0..1 (anti-aliased edges)
cu = uc; cv = vc; r = c(3);
for it = 1:5
    dd = hypot(X - cu, Y - cv);
    inR = dd <= r + 1.5;
    % (a) symmetric exclusion of the glint zone (unbiased if enough is visible)
    mc = round(2*cu - X + 0.5) - (c1 - 1);
    mr = round(2*cv - Y + 0.5) - (r1 - 1);
    inw = mc >= 1 & mc <= nc & mr >= 1 & mr <= nr;
    mirOk = false(nr, nc);
    mirOk(inw) = ~gz(sub2ind([nr nc], mr(inw), mc(inw)));
    use = inR & ~gz & mirOk;
    disc = dd <= max(r, 1);
    if nnz(use & disc) >= 0.5*nnz(disc)
        w = cv_.*use;
    else
        % (b) glint covers the pupil: count glint pixels well inside the disc as pupil
        w = cv_.*(inR & ~gz) + double(gz & dd <= r + 0.15);
    end
    if sum(w(:)) < 0.5, return, end
    cu = sum(w(:).*X(:))/sum(w(:));
    cv = sum(w(:).*Y(:))/sum(w(:));
    dd = hypot(X - cu, Y - cv);
    inR = dd <= r + 1.5;
    disc = dd <= max(r, 1);
    vis = max(0.25, 1 - nnz(disc & gz)/max(1, nnz(disc)));
    r = max(0.5, sqrt(sum(sum(cv_.*(inR & ~gz)))/(pi*vis)));
    r = min(r, 2*c(3) + 1);
end
e.uv = [cu cv];
e.r = r;
e.conf = c(4);
% glints within max(3 px, glintRadK*r) of the pupil centre (corneal reflections)
bright = G > max(D.glintMin, D.glintThr*gmax);
gd = hypot(X - cu, Y - cv);
gm = small_bright(G, bright & gd <= max(3, D.glintRadK*r), D.maxGlintArea);
e.glints = zeros(0, 2);
if any(gm(:))
    [Lg, ng] = cc_label(gm);
    gl = zeros(ng, 3);
    for j = 1:ng
        % sub-pixel centre: weight (I - level) over the spot grown by 1 px, so the
        % dimmer flanks count and the centre is not quantised to the peak pixel
        mj = conv2(double(Lg == j), ones(3), 'same') > 0;
        pk = max(G(mj));
        lvl = bg + 0.3*(pk - bg);
        wj = max(G(mj) - lvl, 0);
        if sum(wj) <= 0, wj = ones(nnz(mj), 1); end
        gu = sum(wj.*X(mj))/sum(wj); gv = sum(wj.*Y(mj))/sum(wj);
        gl(j, :) = [gu, gv, hypot(gu - cu, gv - cv)];
    end
    gl = sortrows(gl, 3);
    e.glints = gl(1:min(D.maxGlints, ng), 1:2);
end
if isempty(e.glints)
    e.gc = [NaN NaN]; e.pccr = [NaN NaN];
else
    e.gc = mean(e.glints, 1); e.pccr = e.uv - e.gc;
end
end

function m = small_bright(G, m, maxArea)
% keep only compact bright spots (glints), not large bright areas (sclera, skin)
[L, n] = cc_label(m);
for j = 1:n
    if nnz(L == j) > maxArea, m(L == j) = false; end
end
end

function Gd = min_down(G, s)
% s x s minimum downsampling (keeps small dark pupils visible)
[H, W] = size(G);
H2 = floor(H/s)*s; W2 = floor(W/s)*s;
G = G(1:H2, 1:W2);
Gd = reshape(min(reshape(G, s, []), [], 1), H2/s, W2);
Gd = reshape(min(reshape(Gd', s, []), [], 1), W2/s, H2/s)';
end

function R = merge_rois(R)
% merge overlapping rectangles [r1 r2 c1 c2]
changed = true;
while changed && size(R, 1) > 1
    changed = false;
    for i = 1:size(R, 1)
        for j = i+1:size(R, 1)
            a = R(i,:); b = R(j,:);
            if a(1) <= b(2) && b(1) <= a(2) && a(3) <= b(4) && b(3) <= a(4)
                R(i,:) = [min(a(1),b(1)) max(a(2),b(2)) min(a(3),b(3)) max(a(4),b(4))];
                R(j,:) = []; changed = true; break
            end
        end
        if changed, break, end
    end
end
end
