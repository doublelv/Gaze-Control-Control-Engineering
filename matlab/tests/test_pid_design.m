function test_pid_design()
%TEST_PID_DESIGN  Camera-axis PID: stable with >= 35 deg phase margin and >= 6 dB gain margin.
P = params();
M = build_axis_model(P.ax(1), P);
C = design_pid(M, P.ax(1), P.Ts);
mg = loop_margins(M, C, P.Ts);
assert(mg.PM >= 35, 'phase margin %.1f deg too low', mg.PM);
assert(mg.GM >= 6,  'gain margin %.1f dB too low', mg.GM);
end
