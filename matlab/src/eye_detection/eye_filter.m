function [st, pHat, vHat] = eye_filter(st, z, dt, P, isNew)
%EYE_FILTER  Constant-velocity Kalman filter on a 3-D position (head / eye).
%   Smooths the measurement, estimates the velocity (for carriage feed-forward),
%   predicts ahead by the camera latency, coasts through blinks. Jumps larger
%   than P.trk.jumpGate sigma (eye saccades shift the pupils by a few mm) move
%   the position estimate but do not create a velocity.
%   st    : state; start with st = [] (initialised by the first measurement)
%   z     : 3x1 measured position [m] (NaN = no measurement)
%   dt    : time since the previous call [s]
%   isNew : true if z comes from a new camera frame (default: z not NaN)
%   pHat  : latency-compensated position,  vHat : velocity [m/s]
if nargin < 5, isNew = ~any(isnan(z)); end
if isempty(st)
    if any(isnan(z)), pHat = nan(3,1); vHat = zeros(3,1); return, end
    st.x = [z'; zeros(1,3)];                     % 2 x 3: [pos; vel] per axis
    st.Pc = repmat({diag([1e-4 1e-2])}, 1, 3);
    isNew = false;                               % already used for the initialisation
end
F  = [1 dt; 0 1];
qa = P.trk.qAcc;
Q  = qa*[dt^3/3 dt^2/2; dt^2/2 dt];
sz = P.cam.sigmaPx*P.cam.zEye/P.cam.f_px;        % measurement noise [m]
Rm = [sz^2, sz^2, (P.cam.zRelErr*P.cam.zEye)^2/4];
for a = 1:3
    x = F*st.x(:,a); Pc = F*st.Pc{a}*F' + Q;
    if isNew && ~isnan(z(a))
        H = [1 0]; S = H*Pc*H' + Rm(a); K = Pc*H'/S;
        innov = z(a) - H*x;
        if abs(innov) > P.trk.jumpGate*sqrt(S) + P.trk.jumpMin
            % jump (eye saccade shifts the pupils, or re-lock): move the position,
            % do NOT turn the jump into a velocity
            x(1) = x(1) + K(1)*innov;
        else
            x = x + K*innov; Pc = (eye(2) - K*H)*Pc;
        end
    end
    st.x(:,a) = x; st.Pc{a} = Pc;
end
pHat = st.x(1,:)' + st.x(2,:)'*P.cam.latency;    % predict to "now"
vHat = st.x(2,:)';
end
