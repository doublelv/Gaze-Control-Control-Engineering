function P = params_local(P)
%PARAMS_LOCAL  Machine-specific overrides. Copy this file to
%   matlab/config/local/params_local.m (that folder is git-ignored) and edit.
%   Called at the end of params.m. The values tuned on the rig (Ts, PID bandwidth,
%   camera latency, dead band, servo zero, ...) are in params.m: keep only what
%   differs on YOUR computer here.
P.link.port      = "COM5";   % Device Manager -> Ports (COM & LPT) -> "STMicroelectronics STLink Virtual COM Port"
% P.cam.deviceName = 'USB Camera';  % only if webcamlist names the camera differently (params.m: 'openaicam')
% P.det.ipd      = 0.062;    % the user's measured interpupillary distance [m]
% P.mot.Jm       = 1.6e-6;   % from run_rig TASK = 'sysid' (params.m has the value of our rig)
end
