function [a, k, J, fitInfo] = sysid_step(t, theta, V, P)
%SYSID_STEP  Identify the camera-axis plant from a REAL open-loop voltage step.
%   Use this once the rig is built, to replace the [PLACEHOLDER] rotor inertia.
%
%   Experiment (carriage in the middle of the rail, endstops armed!):
%     apply a constant voltage V (e.g. 3 V) for ~0.3 s, log the encoder at 1 kHz.
%   Inputs:
%     t     [s]   time vector, step applied at t = 0
%     theta [rad] output-shaft angle from the encoder (700 CPR)
%     V     [V]   step voltage
%     P           params()  (for R, Kt, Ke)
%   Model:  theta'' + a theta' = k V - c*sign(w)   (Coulomb friction c)
%           -> speed w(t) = (k V - c)/a * (1 - exp(-a t))
%   Outputs: a [1/s], k [rad/(V s^2)], J [kg m^2] total at the output shaft.

t = t(:); theta = theta(:) - theta(1);
% Fit the POSITION directly (differentiating 700-CPR encoder data is too noisy):
%   theta(t) = wInf * g(t),  g = s - (1 - exp(-a s))/a,  s = max(t - td, 0)
% The model is linear in wInf, so for every (a, td) on a grid wInf follows from
% least squares; the best grid point is refined with fminsearch. td may be
% slightly negative (the step can start before the first logged sample).
gfun  = @(a, td) max(t - td, 0) - (1 - exp(-a*max(t - td, 0)))/a;
best = [inf 0 0 0];
for a = logspace(log10(3), log10(300), 80)
    for td = linspace(-0.02, 0.05, 36)
        g = gfun(a, td); w = (g'*theta)/max(g'*g, eps);
        c = sum((theta - w*g).^2);
        if c < best(1), best = [c a td w]; end
    end
end
cost = @(q) sum((theta - lsw(gfun(exp(q(1)), q(2)), theta).*gfun(exp(q(1)), q(2))).^2) + 1e6*(abs(q(2) - 0.015) > 0.035);
q = fminsearch(cost, [log(best(2)) best(3)], optimset('TolX', 1e-6, 'TolFun', 1e-12, 'MaxFunEvals', 4000));
a = exp(q(1)); fitInfo.td = q(2);
g = gfun(a, q(2)); wInf = lsw(g, theta); tau = 1/a;
model = @(p) p(2)*gfun(p(1), p(3)); p = [a wInf q(2)];

% torque balance at steady state gives the friction; k from the initial slope
M = build_axis_model(P.ax(1), P);
J = M.Kt/(M.R*k_from(a, M));
k = M.Kt/(M.R*J);

fitInfo.tau = tau; fitInfo.wInf = wInf;
fitInfo.thetaModel = model(p);
fitInfo.rmsErr = sqrt(mean((theta - fitInfo.thetaModel).^2));
fitInfo.frictionV = V - wInf*M.Ke;           % voltage lost to friction + R*i0
fprintf('tau = %.1f ms, dead time %.1f ms, a = %.1f 1/s, J = %.2e kg m^2, fit RMS = %.4f rad\n', ...
        tau*1e3, fitInfo.td*1e3, a, J, fitInfo.rmsErr);
fprintf('-> set P.mot.Jm = %.2e in matlab/config/local/params_local.m, then rerun run_simulation for new PID gains\n', ...
        (J - P.ax(1).JL)/P.mot.N^2);
end

function k = k_from(a, M)
% a = (b + Kt Ke / R)/J  with b ~ 0  ->  J = Kt Ke/(R a)  ->  k = Kt/(R J) = a/Ke
k = a/M.Ke;
end

function w = lsw(g, theta)
w = (g'*theta)/max(g'*g, eps);
end
