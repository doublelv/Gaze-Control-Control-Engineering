function [box, fd] = face_find(fd, img, P, prev)
%FACE_FIND  Face bounding box in a camera frame (visible or IR light, no LEDs needed).
%   fd   : detector state, [] on the first call (created here)
%   img  : H x W uint8 frame (full resolution)
%   prev : previous box [x y w h] or [] -> pick the face nearest to it,
%          otherwise the largest face
%   box  : [x y w h] in full-resolution pixels (MATLAB convention, x = column), [] if none
%   Uses vision.CascadeObjectDetector (Viola-Jones, Computer Vision Toolbox) on
%   an image downscaled by P.face.scale. If P.face.detectFcn is a function handle
%   (tests, no toolbox), it is called as boxes = detectFcn(smallImage) instead.
if nargin < 4, prev = []; end
k = max(1, round(1/P.face.scale)); s = 1/k;          % integer decimation (no toolbox needed)
if isempty(fd)
    if ~isempty(P.face.detectFcn)
        fd = P.face.detectFcn;
    else
        fd = vision.CascadeObjectDetector('FrontalFaceCART', ...
                 'MinSize', max(20, round(P.face.minSizePx*s))*[1 1], 'MergeThreshold', 4);
    end
end
small = img(1:k:end, 1:k:end);
if isa(fd, 'function_handle'), B = fd(small); else, B = step(fd, small); end
box = [];
if isempty(B), return, end
B = double(B);
B(:, 1:2) = (B(:, 1:2) - 1)*k + 1;              % back to full-resolution pixels
B(:, 3:4) = B(:, 3:4)/s;
if isempty(prev)
    [~, i] = max(B(:,3).*B(:,4));
else
    c  = B(:,1:2) + B(:,3:4)/2;
    cp = prev(1:2) + prev(3:4)/2;
    [~, i] = min(sum((c - cp).^2, 2));
end
box = B(i, :);
end
