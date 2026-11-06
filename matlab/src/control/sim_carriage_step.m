function [t, xc, V] = sim_carriage_step(P, M, C, xStep, Tloop, Tend, vmax)
%SIM_CARRIAGE_STEP  Carriage step on the nonlinear model (friction, PWM, encoder,
%   saturation, one-loop command delay). PID C designed for P.Ts runs every Tloop.
%   Tloop: scalar [s] or a vector of loop periods (e.g. measured L.loopMs/1e3, repeated).
%   vmax : reference slew limit [rad/s] (default P.ax(1).vmax; Inf = true step)
if nargin < 7 || isempty(vmax), vmax = P.ax(1).vmax; end
n0 = round(Tend/mean(Tloop));
if isscalar(Tloop), dts = repmat(Tloop, 1, n0); else, dts = Tloop(1 + mod(0:n0-1, numel(Tloop))); end
t = [0 cumsum(dts(1:end-1))]; n = numel(t);
P1 = P; P1.Ts = 1e-4; P1.nSub = 1; Mt = build_axis_model(P.ax(1), P1);
x = zeros(3,1); Vapp = 0; refPrev = 0; xc = zeros(1, n); V = zeros(1, n);
st = struct('I', 0, 'D', 0, 'yPrev', 0);
for k = 1:n
    y = round(x(1)/M.q)*M.q; xc(k) = x(1)*M.r;
    ref = xStep/M.r;
    dmax = vmax*dts(k); ref = refPrev + min(max(ref - refPrev, -dmax), dmax); refPrev = ref;
    [Vm, st] = pid_step(st, C, M, P, ref, y, 0, dts(k));
    V(k) = Vm;
    for s = 1:max(1, round(dts(k)/1e-4))
        x = Mt.Ad*x + Mt.Bd*[Vapp; Mt.Tc*tanh(x(2)/0.02)];
    end
    Vapp = round(Vm/M.Vq)*M.Vq;
end
end
