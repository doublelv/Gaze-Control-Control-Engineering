function pEye = eye_trajectory(P, t)
%EYE_TRAJECTORY  Test scenario: true 3-D eye position [m], 3 x numel(t).
%   Smooth head sway in x, y, z plus one sudden head shift at tJump.
tr = P.traj; n = numel(t);
pEye = repmat(tr.p0, 1, n);
pEye(1,:) = pEye(1,:) + tr.Ax*sin(2*pi*tr.fx*t);
pEye(2,:) = pEye(2,:) + tr.Ay*sin(2*pi*tr.fy*t);
pEye(3,:) = pEye(3,:) + tr.Az*sin(2*pi*tr.fz*t);
pEye = pEye + tr.jump*(t >= tr.tJump);
end
