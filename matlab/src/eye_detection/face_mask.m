function imgE = face_mask(img, box)
%FACE_MASK  Keep only the upper part of the face box (eyes), fill the rest with
%   a flat grey, so the pupil detector cannot lock onto the background.
%   box = [] -> the whole frame is flat (no face, no eye search).
fillV = uint8(128);
if isempty(box), imgE = img; imgE(:) = fillV; return, end
[Hh, Ww] = size(img);
r1 = max(1, floor(box(2))); r2 = min(Hh, ceil(box(2) + 0.65*box(4)));
c1 = max(1, floor(box(1))); c2 = min(Ww, ceil(box(1) + box(3)));
inside = img(r1:r2, c1:c2);
fillV = uint8(median(double(inside(:))));
imgE = img; imgE(:) = fillV;
imgE(r1:r2, c1:c2) = inside;
end
