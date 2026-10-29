function [V, st] = pid_step(st, C, M, P, ref, y, Vff, dt)
%PID_STEP  One sample of the camera-axis PID (same law as simulate_rig.m).
%   st  : state struct, initialise with st = struct('I',0,'D',0,'yPrev',y0)
%   ref, y : output-shaft angle reference and measurement [rad]
%   Vff : optional feed-forward voltage [V] (velocity FF)
%   dt  : optional measured sample time [s] (default P.Ts); clipped to [0.5, 3]*P.Ts
%   V   : motor voltage command [V], saturated to the PSU voltage
if nargin < 7 || isempty(Vff), Vff = 0; end
if nargin < 8 || isempty(dt), dt = P.Ts; end
Ts = min(max(dt, 0.5*P.Ts), 3*P.Ts);
e  = ref - y;
st.D = (C.Tf*st.D - C.Kd*(y - st.yPrev))/(C.Tf + Ts);   % filtered D on measurement
st.yPrev = y;
Vu = C.Kp*(P.bSP*ref - y) + st.I + st.D + Vff;          % set-point weighted P + FF
V  = min(max(Vu, -M.Vmax), M.Vmax);
if Vu == V || sign(e) ~= sign(Vu)                       % conditional integration
    st.I = st.I + C.Ki*Ts*e;
end
end
