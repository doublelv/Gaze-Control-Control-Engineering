function [root, mroot] = setup_paths()
%SETUP_PATHS  Add src/*, config and config/local to the MATLAB path.
%   root  : folder for outputs (artifacts/): the repository root, or the MATLAB
%           folder itself when it is used stand-alone (eye_tracking_sim)
%   mroot : the MATLAB folder (contains config/, src/, scripts/, data/)
%   Call once at the start of every script. Has no hardware effects.
here = fileparts(mfilename('fullpath'));          % .../scripts
mroot = fileparts(here);                          % .../matlab (or eye_tracking_sim)
root  = fileparts(mroot);                         % repository root
if ~exist(fullfile(root, 'interface'), 'dir'), root = mroot; end   % stand-alone copy
addpath(fullfile(mroot, 'config'));
loc = fullfile(mroot, 'config', 'local');          % git-ignored machine settings
if ~exist(loc, 'dir'), mkdir(loc); end
addpath(loc);
mods = {'camera', 'eye_detection', 'gaze_estimation', 'control', 'communication', 'visualization'};
for i = 1:numel(mods)
    addpath(fullfile(mroot, 'src', mods{i}));
end
addpath(here);
end
