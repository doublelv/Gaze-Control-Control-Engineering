function out = gaze_calib_file(P, cmd, M)
%GAZE_CALIB_FILE  Save / load the gaze calibration in matlab/config/local/
%   (git-ignored: calibration data is personal and must not be committed).
%   gaze_calib_file(P, 'save', M)    M = gaze_calib_load: out = gaze_calib_file(P, 'load')
here = fileparts(mfilename('fullpath'));
f = fullfile(here, '..', '..', 'config', 'local', P.gcal.file);
switch cmd
    case 'save'
        d = fileparts(f); if ~exist(d, 'dir'), mkdir(d); end
        gazeCal = M; save(f, 'gazeCal'); out = f;
    case 'load'
        out = [];
        if exist(f, 'file'), s = load(f); out = s.gazeCal; end
end
end
