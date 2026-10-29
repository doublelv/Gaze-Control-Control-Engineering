function [uv, inFront] = cam_project(P, X, xCar)
%CAM_PROJECT  World point(s) X (3xN) -> full-image pixel coordinates (2xN),
%   camera on the carriage at (xCar, 0, 0) + P.geo.cam, looking +z, no rotation.
%   u to the right, v DOWN, origin at the top-left pixel corner (MATLAB image
%   pixel (r, c) has its centre at (u, v) = (c - 0.5, r - 0.5)).
C = P.geo.cam + [xCar; 0; 0];
d = X - C;
inFront = d(3,:) > 1e-3;
uv = [P.cam.W/2 + P.cam.f_px*d(1,:)./d(3,:);
      P.cam.H/2 - P.cam.f_px*d(2,:)./d(3,:)];
end
