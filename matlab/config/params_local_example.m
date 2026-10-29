function P = params_local(P)
%PARAMS_LOCAL  Machine-specific overrides. Copy this file to
%   matlab/config/local/params_local.m (that folder is git-ignored) and edit.
%   Called at the end of params.m.
P.link.port      = "COM5";   % Device Manager -> Ports (COM & LPT) -> "STMicroelectronics STLink Virtual COM Port"
P.cam.deviceName = '';       % part of the name shown by webcamlist, e.g. 'USB Camera'
% P.det.ipd      = 0.062;    % the user's measured interpupillary distance [m]
% P.mot.Jm       = 1.6e-6;   % from run_rig TASK = 'sysid'
end
