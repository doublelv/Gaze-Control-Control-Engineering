function [Rref, pEye, xCarRef] = coordinator_setpoints(P, t)
%COORDINATOR_SETPOINTS  References from the TRUE eye path (ideal camera).
%   Used as the "perfect sensor" baseline. The camera-based version is
%   camera_measure -> eye_filter -> refs_from_eye (see main.m section 4).
pEye = eye_trajectory(P, t);
[Rref, xCarRef] = refs_from_eye(P, pEye, t(2) - t(1));
end
