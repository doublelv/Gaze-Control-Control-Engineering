function out = simulate_rig(P, M, C, t, Rref, RrefDot)
%SIMULATE_RIG  Closed-loop simulation of all axes together.
%   M, C : struct arrays from build_axis_model / design_pid
%   t    : 1xn time vector (step Ts),  Rref : nAx x n angle references [rad]
%   RrefDot (optional): nAx x n reference velocities [rad/s] -> velocity
%          feed-forward on 'dc' axes (back-EMF + friction compensation)
%
%   'dc' axes   : discrete PID (set-point weighted P, filtered D on measurement),
%                 anti-windup, PWM voltage quantisation, saturation at PSU
%                 voltage, encoder quantisation, compute delay, Coulomb
%                 friction, gravity + feed-forward, cross-axis coupling.
%   'servo' axes: MCU sends a pulse width every 20 ms (quantised to usRes);
%                 the servo moves toward it with its own speed-limited loop
%                 and stops inside its dead band. No feedback to the MCU.

nAx = numel(M); n = numel(t); Ts = P.Ts;
if nargin < 6 || isempty(RrefDot), RrefDot = zeros(size(Rref)); end
wEps = 0.02;
isDC = arrayfun(@(m) strcmp(m.type, 'dc'), M);
s = P.srv;
pulseCenter = (s.pulseMin + s.pulseMax)/2;
radPerUs = s.range/(s.pulseMax - s.pulseMin);

x = zeros(3, nAx);
x(1,:) = Rref(:,1)';
Iint = zeros(nAx,1); Dterm = zeros(nAx,1); yPrev = zeros(nAx,1);
Vhold = zeros(nAx,1); acc = zeros(nAx,1);
thCmd = Rref(:,1);                       % servo latched command angle
pulse = NaN(nAx,1);
frameN = round(s.framePer/Ts);
for i = find(isDC)
    x(3,i)   = M(i).mgl*cos(x(1,i))/M(i).Kt;
    Iint(i)  = C(i).Kp*(1 - P.bSP)*Rref(i,1);
    yPrev(i) = M(i).q*round(x(1,i)/M(i).q);
    Vhold(i) = M(i).R*x(3,i);
end

out.t = t; out.ref = Rref;
out.th = zeros(nAx,n); out.w = zeros(nAx,n); out.i = NaN(nAx,n);
out.V  = NaN(nAx,n); out.pulse = NaN(nAx,n);

for k = 1:n
    % ---------------- controllers ----------------
    Vcmd = zeros(nAx,1);
    for i = 1:nAx
        if isDC(i)
            y = M(i).q*round(x(1,i)/M(i).q);
            e = Rref(i,k) - y;
            Dterm(i) = (C(i).Tf*Dterm(i) - C(i).Kd*(y - yPrev(i)))/(C(i).Tf + Ts);
            yPrev(i) = y;
            Vff = 0;
            if P.useGravFF, Vff = M(i).R*M(i).mgl*cos(Rref(i,k))/M(i).Kt; end
            wr  = RrefDot(i,k);                           % velocity feed-forward
            Vff = Vff + M(i).Ke*wr + M(i).R*M(i).Tc/M(i).Kt*tanh(wr/0.05);
            Vu = C(i).Kp*(P.bSP*Rref(i,k) - y) + Iint(i) + Dterm(i) + Vff;
            Vs = min(max(Vu, -M(i).Vmax), M(i).Vmax);
            if Vu == Vs || sign(e) ~= sign(Vu)
                Iint(i) = Iint(i) + C(i).Ki*Ts*e;
            end
            Vcmd(i) = M(i).Vq*round(Vs/M(i).Vq);          % PWM duty quantisation
        elseif mod(k-1, frameN) == 0                       % new 50 Hz servo frame
            pw = pulseCenter + Rref(i,k)/radPerUs;
            pw = s.usRes*round(pw/s.usRes);
            pw = min(max(pw, s.pulseMin), s.pulseMax);
            pulse(i) = pw;
            thCmd(i) = (pw - pulseCenter)*radPerUs;
        end
    end
    if P.calcDelay, Vapp = Vhold; Vhold = Vcmd; else, Vapp = Vcmd; end

    out.th(:,k) = x(1,:)'; out.w(:,k) = x(2,:)';
    out.i(isDC,k) = x(3,isDC)'; out.V(isDC,k) = Vapp(isDC);
    out.pulse(:,k) = pulse;

    % ---------------- plants ----------------
    for ss = 1:P.nSub
        wOld = x(2,:)';
        for i = 1:nAx
            if isDC(i)
                tauG = M(i).mgl*cos(x(1,i));
                tauF = M(i).Tc*tanh(x(2,i)/wEps);
                tauC = 0;
                for j = 1:nAx, tauC = tauC + P.kc(i,j)*M(j).J*acc(j); end
                x(:,i) = M(i).Ad*x(:,i) + M(i).Bd*[Vapp(i); tauG + tauF + tauC];
            else
                err = thCmd(i) - x(1,i);
                if abs(err) < M(i).deadAng/2
                    wT = 0;
                else
                    wT = min(max(s.Kin*err, -M(i).wmaxEff), M(i).wmaxEff);
                end
                x(2,i) = x(2,i) + (wT - x(2,i))*M(i).dt/s.tauW;
                x(1,i) = x(1,i) + x(2,i)*M(i).dt;
            end
        end
        acc = (x(2,:)' - wOld)/M(1).dt;
    end
end
end
