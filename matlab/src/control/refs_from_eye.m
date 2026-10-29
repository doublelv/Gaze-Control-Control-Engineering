function [Rref, xCarRef] = refs_from_eye(P, pEye, dt)
%REFS_FROM_EYE  Eye position(s) -> references for the 3 axes (the MIMO coordinator).
%   pEye : 3 x n eye positions [m] (true, or estimated from the camera)
%   dt   : sample time for the slew-rate limit; omit/[] for no limit (single sample)
%   Axis 1: carriage x follows the eye x, clamped to the travel, theta = x/r.
%   Axis 2-3: illuminators ride on the carriage and tilt toward the eye; they
%             use the carriage REFERENCE (feed-forward), not its measured position.
if nargin < 3, dt = []; end
r = P.belt.r; n = size(pEye, 2);
Rref = zeros(3, n);

xRaw = min(max(pEye(1,:) - P.geo.cam(1), P.belt.xMin), P.belt.xMax);
Rref(1,:) = xRaw/r;
if ~isempty(dt), Rref(1,:) = slew(Rref(1,:), P.ax(1).vmax*dt); end
xCarRef = Rref(1,:)*r;

offs = [P.geo.illumA_off P.geo.illumB_off];
for j = 1:2
    Lx = P.geo.cam(1) + xCarRef + offs(1,j);
    Ly = P.geo.cam(2) + offs(2,j);
    Lz = P.geo.cam(3) + offs(3,j);
    Rref(j+1,:) = atan2(pEye(2,:) - Ly, hypot(pEye(1,:) - Lx, pEye(3,:) - Lz));
    if ~isempty(dt), Rref(j+1,:) = slew(Rref(j+1,:), P.ax(j+1).vmax*dt); end
end
end

function y = slew(u, dmax)
y = u;
for k = 2:numel(u)
    y(k) = y(k-1) + min(max(u(k) - y(k-1), -dmax), dmax);
end
end
