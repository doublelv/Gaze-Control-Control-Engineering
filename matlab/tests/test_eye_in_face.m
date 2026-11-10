function test_eye_in_face()
%TEST_EYE_IN_FACE  Iris finder inside a face box: drawn face with two irises,
%   a lash line, hair at the box edge and a closed-eye case.
P = params();
W = 640; H = 480;
box = [220 140 200 200];                         % [x y w h]
ev = box(2) + P.face.eyeLevel*box(4);            % eye row
eu = box(1) + [0.69 0.31]*box(3);                % [image right, image left] iris columns
[X, Y] = meshgrid((1:W) - 0.5, (1:H) - 0.5);

img = 40*ones(H, W);                             % dark background
img(box(2):box(2)+box(4), box(1):box(1)+box(3)) = 170;   % skin
img(:, box(1)+round(0.88*box(3)):box(1)+box(3)) = 30;    % hair inside the box edge
for e = 1:2
    img(abs(Y - ev + 6) < 1 & abs(X - eu(e)) < 18) = 15;     % lash line above the eye
    img(hypot(X - eu(e), Y - ev) < 5) = 50;                  % iris
end
d = eye_in_face(uint8(img), box, P);
assert(numel(d.eyes) == 2 && d.pair, 'open eyes: expected a pair');
for e = 1:2
    err = hypot(d.eyes(e).uv(1) - eu(e), d.eyes(e).uv(2) - ev);
    assert(err < 1.5, 'eye %d off by %.2f px', e, err);
end

assert(all(arrayfun(@(x) isempty(x.glints), d.eyes)), 'no LED reflection drawn, but a glint was found');

% IR glint (LED reflection) on the image-right iris -> found there only
imgG = img;
imgG(abs(Y - ev + 1) < 1 & abs(X - eu(1) - 1) < 1) = 255;
dG = eye_in_face(uint8(imgG), box, P);
assert(numel(dG.eyes) == 2, 'glint: eyes lost');
assert(size(dG.eyes(1).glints, 1) == 1 && isempty(dG.eyes(2).glints), 'glint: expected on eye 1 only');
assert(hypot(dG.eyes(1).gc(1) - (eu(1) + 1), dG.eyes(1).gc(2) - (ev - 1)) < 1.5, 'glint position off');

% closed eyes: only the thin lash lines are left -> no eyes
img2 = 40*ones(H, W);
img2(box(2):box(2)+box(4), box(1):box(1)+box(3)) = 170;
for e = 1:2, img2(abs(Y - ev) < 1 & abs(X - eu(e)) < 18) = 15; end
d2 = eye_in_face(uint8(img2), box, P);
assert(isempty(d2.eyes), 'closed eyes: expected none, got %d', numel(d2.eyes));

% no face box -> nothing
d3 = eye_in_face(uint8(img), [], P);
assert(isempty(d3.eyes) && ~d3.pair, 'no box: expected nothing');
end
