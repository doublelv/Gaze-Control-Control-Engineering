function test_pid_design()
%TEST_PID_DESIGN  Camera-axis PID: stable with >= 35 deg phase margin and >= 6 dB
%   gain margin at the design period; full PID (Kd > 0) from the phase-margin rule,
%   which reaches P.ax(1).pmTarget at the measured loop period P.ax(1).TsPM.
P = params();
M = build_axis_model(P.ax(1), P);
C = design_pid(M, P.ax(1), P.Ts);
mg = loop_margins(M, C, P.Ts);
assert(mg.PM >= 35, 'phase margin %.1f deg too low', mg.PM);
assert(mg.GM >= 6,  'gain margin %.1f dB too low', mg.GM);
assert(C.Kp > 0 && C.Ki > 0 && C.Kd > 0, 'not a full PID: Kp %.3f Ki %.3f Kd %.3f', C.Kp, C.Ki, C.Kd);
m46 = loop_margins(M, C, P.ax(1).TsPM);
assert(m46.PM >= P.ax(1).pmTarget && m46.PM < P.ax(1).pmTarget + 1, 'PM at %.0f ms: %.1f deg', P.ax(1).TsPM*1e3, m46.PM);
Cpi = C; Cpi.Kd = 0; mpi = loop_margins(M, Cpi, P.ax(1).TsPM);
assert(m46.PM > mpi.PM + 5, 'D term adds only %.1f deg', m46.PM - mpi.PM);
end
