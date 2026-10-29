function sim = virtual_mcu_init(P)
%VIRTUAL_MCU_INIT  State of the simulated Nucleo + FIT0186/belt carriage.
%   Same behaviour as the firmware: endstop gating, watchdog, status flags.
sim.M = build_axis_model(P.ax(1), P);
sim.x = zeros(3,1);
sim.x(1) = (P.belt.xMin + 0.05 + 0.1*rand)/sim.M.r;   % unknown start position
sim.cntOffset = int32(round(1e4*rand));               % encoder does not start at 0
sim.t = 0; sim.duty = 0; sim.servoUs = P.srv.zeroUs; sim.led = 0;
sim.lastCmdT = 0; sim.wdogTrip = false; sim.rxErrors = 0; sim.rxErrFlag = false;
sim.wdogArmed = false;
end
