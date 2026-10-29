function [sim, reply] = virtual_mcu_step(sim, pkt, P)
%VIRTUAL_MCU_STEP  Simulated firmware: parse one frame, advance the plant by
%   one control period P.Ts, return the reply frame (same bytes as the real one).
C = proto_const(); M = sim.M;
[msg, ok] = proto_parse(pkt);
reply = uint8([]);
if ~ok
    sim.rxErrors = sim.rxErrors + 1; sim.rxErrFlag = true;
    sim = advance(sim, P, M); return               % firmware sends nothing
end
seq = msg.payload(end);
switch msg.type
    case C.T_HELLO
        reply = proto_frame(C.T_HELLO_REPLY, [uint8([C.version 1 0]) seq typecast(uint32(sim.t*1e6), 'uint8')]);
        return
    case C.T_CMD
        p = msg.payload;
        sim.duty = double(typecast(p(1:2), 'int16'));
        sim.servoUs = double([typecast(p(3:4), 'uint16') typecast(p(5:6), 'uint16')]);
        sim.led = p(7);
        sim.lastCmdT = sim.t; sim.wdogArmed = true;
    case C.T_STOP
        sim.duty = 0; sim.led = 0; sim.lastCmdT = sim.t;
end
sim = advance(sim, P, M);
xCar = sim.x(1)*M.r;
eMin = P.endstops(1) && xCar <= P.belt.xMin;
eMax = P.endstops(2) && xCar >= P.belt.xMax;
limited = (eMin && sim.duty < 0) || (eMax && sim.duty > 0);
flags = C.F_END_MIN*eMin + C.F_END_MAX*eMax + C.F_WDOG*sim.wdogTrip + ...
        C.F_LIMITED*limited + C.F_RX_ERR*sim.rxErrFlag;
fb = struct('count', int32(round(sim.x(1)/M.q)) + sim.cntOffset, 'flags', flags, ...
            'seq', seq, 'tUs', mod(round(sim.t*1e6), 2^32), 'rxErrors', mod(sim.rxErrors, 65536));
reply = encode_state(fb);
sim.wdogTrip = false; sim.rxErrFlag = false;
end

function sim = advance(sim, P, M)
% one control period of plant time, with the firmware safety rules
duty = sim.duty;
if sim.wdogArmed && sim.t - sim.lastCmdT > P.link.watchdog
    duty = 0; sim.led = 0;
    if sim.duty ~= 0, sim.wdogTrip = true; end
    sim.duty = 0;
end
for s = 1:P.nSub
    xCar = sim.x(1)*M.r;
    eMin = P.endstops(1) && xCar <= P.belt.xMin;
    eMax = P.endstops(2) && xCar >= P.belt.xMax;
    d = duty; if (eMin && d < 0) || (eMax && d > 0), d = 0; end
    V = d/P.drv.pwmCounts*M.Vmax;
    tauF = M.Tc*tanh(sim.x(2)/0.02);
    sim.x = M.Ad*sim.x + M.Bd*[V; tauF];
    xc = sim.x(1)*M.r;                                   % mechanical hard stops
    xHi = P.belt.xMax + (P.belt.xMin - P.belt.xStop);
    if xc < P.belt.xStop || xc > xHi
        sim.x(1) = min(max(xc, P.belt.xStop), xHi)/M.r; sim.x(2) = 0;
    end
end
sim.t = sim.t + P.Ts;
end
