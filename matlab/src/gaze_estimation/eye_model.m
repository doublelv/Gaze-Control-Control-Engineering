function E = eye_model(P, center, target)
%EYE_MODEL  3-D geometry of one eye looking at a target (world frame, metres).
%   center : eyeball rotation centre (3x1),  target : fixation point (3x1)
%   E.g    : optical axis (unit, from the eye toward the target)
%   E.c    : cornea centre of curvature  (5.2 mm in front of the rotation centre)
%   E.p    : pupil centre                (P.eye.K = 4.2 mm in front of E.c)
%   E.R    : cornea radius (P.eye.R)
%   Simplified Le Grand eye; optical axis only (no kappa angle).
E.g = (target - center)/norm(target - center);
E.c = center + 5.2e-3*E.g;
E.p = E.c + P.eye.K*E.g;
E.R = P.eye.R;
end
