function M = build_axis_model(ax, P)
%BUILD_AXIS_MODEL  Plant model for one axis.
%
%  type 'dc'   : DC motor + gearbox + load, reflected to the output shaft
%     States x = [theta; omega; i],  inputs u = [V; tau_load]
%     J dw/dt = Kt_e*i - b*w - tau_load,   L di/dt = V - R*i - Ke_e*w
%     J = JL + N^2 Jm, b = bL + N^2 bm, Kt_e = eta N Kt, Ke_e = N Ke
%     Linear (belt) axis: JL = m r^2, carriage position x = r*theta
%
%  type 'servo': hobby servo (SER0006) treated as a black box with its own
%     internal position loop: speed-limited, dead band, 50 Hz pulse frames.
%     States x = [theta; omega; 0]

mot = P.mot; N = mot.N; g = P.g;
% same field set for both types so they fit in one struct array
f = {'J','b','Kt','Ke','R','L','Vmax','Vq','q','A','B','Ad','Bd','a','k', ...
     'Istall','deadAng','wmaxEff','torqueMargin'};
for n = 1:numel(f), M.(f{n}) = NaN; end
M.name = ax.name;
M.r    = ax.r;
M.type = ax.type;
M.mgl  = ax.m*g*ax.l;
M.Tc   = ax.Tc;
M.dt   = P.Ts/P.nSub;

if strcmp(ax.type, 'servo')
    s = P.srv;
    M.J = ax.JL;
    M.q = s.usRes*s.range/(s.pulseMax - s.pulseMin);          % [rad] per us step
    M.deadAng = s.deadUs*s.range/(s.pulseMax - s.pulseMin);   % [rad]
    M.wmaxEff = s.wmax*max(0, 1 - M.mgl/s.tauStall);          % linear speed-torque
    M.torqueMargin = s.tauStall/max(M.mgl, eps);
    M.Vmax = NaN;
    return
end

M.J  = ax.JL + N^2*mot.Jm;
M.b  = ax.bL + N^2*mot.bm;
M.Kt = mot.eta*N*mot.Kt;              % gearbox efficiency on torque
M.Ke = N*mot.Ke;
M.R  = mot.R;
M.L  = mot.L;
M.Vmax = mot.Vmax;
M.Vq   = mot.Vmax/P.drv.pwmCounts;     % PWM voltage step
M.q    = 2*pi/(mot.cpr*N);              % encoder resolution at output [rad]

A = [0  1            0;
     0 -M.b/M.J      M.Kt/M.J;
     0 -M.Ke/M.L    -M.R/M.L];
B = [0      0;
     0     -1/M.J;
     1/M.L  0];
M.A = A; M.B = B;

% Exact zero-order-hold discretisation of the linear part over one sub-step
Phi  = expm([A B; zeros(2,5)]*M.dt);
M.Ad = Phi(1:3,1:3);
M.Bd = Phi(1:3,4:5);

% Reduced 2nd-order model (L -> 0) used for PID design: theta'' + a theta' = k V
M.a = (M.b + M.Kt*M.Ke/M.R)/M.J;
M.k = M.Kt/(M.R*M.J);
M.Istall = M.Vmax/M.R;                  % worst-case current at full voltage
end
