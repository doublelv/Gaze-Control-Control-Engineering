function B = face_stub_synthetic(img)
%FACE_STUB_SYNTHETIC  Stand-in for vision.CascadeObjectDetector in tests (no
%   toolbox needed): the synthetic face is the bright blob on a dark background.
%   Returns [x y w h] like the real detector, [] if no face.
m = img > 70;
r = find(any(m, 2)); c = find(any(m, 1));
if numel(r) < 5 || numel(c) < 5, B = zeros(0, 4); return, end
B = [c(1) r(1) c(end) - c(1) + 1 r(end) - r(1) + 1];
end
