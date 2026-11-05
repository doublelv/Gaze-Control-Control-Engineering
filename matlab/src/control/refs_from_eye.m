function [Rref, xCarRef] = refs_from_eye(P, pEye, dt)
%REFS_FROM_EYE  Eye position(s) -> references for the 3 axes (the MIMO coordinator).
%   pEye : 3 x n eye positions [m] (true, or estimated from the camera)
%   dt   : sample time for the slew-rate limit; omit/[] for no limit (single sample)
%   Axis 1: carriage x follows the eye x, clamped to the travel, theta = x/r.
%   Axis 2-3: the servo LEDs turn toward the eye (P.led.aim, see led_aim.m).
%             Frame-mounted LEDs (as built) have a fixed position; carriage-
%             mounted ones use the carriage REFERENCE (feed-forward), not its
%             measured position.
if nargin < 3, dt = []; end
r = P.belt.r; n = size(pEye, 2);
Rref = zeros(3, n);

xRaw = min(max(pEye(1,:) - P.geo.cam(1), P.belt.xMin), P.belt.xMax);
Rref(1,:) = xRaw/r;
if ~isempty(dt), Rref(1,:) = slew(Rref(1,:), P.ax(1).vmax*dt); end
xCarRef = Rref(1,:)*r;

for j = 1:2
    i = find(P.led.servo == j, 1);
    if isempty(i), continue, end
    if P.led.onFrame(i)
        L = led_world_pos(P, 0);
        Rref(j+1,:) = led_aim(P, L(:,i), pEye);
    else
        for k = 1:n
            L = led_world_pos(P, xCarRef(k));
            Rref(j+1,k) = led_aim(P, L(:,i), pEye(:,k));
        end
    end
    if ~isempty(dt), Rref(j+1,:) = slew(Rref(j+1,:), P.ax(j+1).vmax*dt); end
end
end

function y = slew(u, dmax)
y = u;
for k = 2:numel(u)
    y(k) = y(k-1) + min(max(u(k) - y(k-1), -dmax), dmax);
end
end
