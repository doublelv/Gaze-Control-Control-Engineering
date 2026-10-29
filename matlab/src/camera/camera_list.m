function names = camera_list()
%CAMERA_LIST  Print the connected cameras and their resolutions (no side effects).
if ~exist('webcamlist', 'file')
    fprintf('Install "MATLAB Support Package for USB Webcams" first.\n'); names = {}; return
end
names = webcamlist;
for i = 1:numel(names)
    c = webcam(i);
    fprintf('%d: %s\n   resolutions: %s\n', i, names{i}, strjoin(c.AvailableResolutions, ', '));
    clear c
end
if isempty(names), fprintf('No camera found.\n'); end
end
