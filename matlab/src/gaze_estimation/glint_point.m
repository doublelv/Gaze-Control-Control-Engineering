function Pg = glint_point(c, R, L, O)
%GLINT_POINT  Point on a spherical cornea (centre c, radius R) where light
%   from source L is reflected into the camera centre O (law of reflection).
%   All 3x1 world coordinates [m]. Solved in the plane through c, L, O.
u1 = (O - c)/norm(O - c);
v  = L - c; v = v - (v'*u1)*u1;
if norm(v) < 1e-12, Pg = c + R*u1; return, end      % source on the camera axis
u2 = v/norm(v);
phiL = atan2((L - c)'*u2, (L - c)'*u1);
f = @(phi) res(phi, c, R, L, O, u1, u2);
% bisection (no toolbox, robust): f(0) > 0, f(phiL) < 0
a = 0; b = phiL; fa = f(a);
for it = 1:40
    m = 0.5*(a + b); fm = f(m);
    if sign(fm) == sign(fa), a = m; fa = fm; else, b = m; end
end
phi = 0.5*(a + b);
Pg = c + R*(cos(phi)*u1 + sin(phi)*u2);
end

function r = res(phi, c, R, L, O, u1, u2)
n  = cos(phi)*u1 + sin(phi)*u2;
Pp = c + R*n;
r  = n'*(O - Pp)/norm(O - Pp) - n'*(L - Pp)/norm(L - Pp);
end
