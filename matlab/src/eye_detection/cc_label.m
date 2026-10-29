function [L, n] = cc_label(mask)
%CC_LABEL  8-connected component labelling (base MATLAB, no toolbox).
%   Vectorised: every true pixel starts with its own label, labels spread as
%   the minimum over neighbour pairs, with pointer jumping; only the true
%   pixels are processed, so it is fast for sparse masks.
%   L: label image (0 = background, 1..n numbered in column-major order of the
%   first pixel), n: number of components.
[H, W] = size(mask);
L = zeros(H, W);
idx = find(mask);
N = numel(idx);
n = 0;
if N == 0, return, end
pos = zeros(H, W); pos(idx) = 1:N;
[r, c] = ind2sub([H W], idx);
I = []; J = [];
for d = [1 0; 1 1; 0 1; -1 1]'                       % 4 forward offsets = 8-connectivity
    rr = r + d(1); cc = c + d(2);
    ok = rr >= 1 & rr <= H & cc >= 1 & cc <= W;
    q = zeros(N, 1); q(ok) = pos(sub2ind([H W], rr(ok), cc(ok)));
    k = find(q > 0);
    I = [I; k]; J = [J; q(k)]; %#ok<AGROW>
end
lab = (1:N)';
if ~isempty(I)
    A = [I; J]; B = [J; I];                           % both directions
    while true
        m = accumarray(A, lab(B), [N 1], @min, Inf);
        new = min(lab, m);
        new = new(new);                               % pointer jumping
        new = new(new);
        if isequal(new, lab), break, end
        lab = new;
    end
end
[~, ~, comp] = unique(lab);                           % 1..n in order of the root pixel
n = max(comp);
L(idx) = comp;
end
