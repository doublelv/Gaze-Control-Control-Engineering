function cam = camera_open(P, scene)
%CAMERA_OPEN  Open a frame source. P.cam.source selects it:
%   'webcam'    : the OV9281 (or any USB camera) via the MATLAB Support Package
%                 for USB Webcams. P.cam.deviceName picks the camera by name
%                 ('' = first one). Resolution 1280x720 if available.
%   'video'     : recorded file P.cam.videoFile (VideoReader)
%   'images'    : folder P.cam.imageDir with PNG/JPG frames (sorted by name)
%   'synthetic' : rendered face (synth_render), scene = synth_scene(...) or [] for 'demo'
%   cam fields: source, W, H, nRead, t0 (tic), plus source-specific handles.
%   Errors with a clear message when the source cannot be opened.
cam = struct('source', P.cam.source, 'W', P.cam.W, 'H', P.cam.H, 'nRead', 0, ...
             't0', tic, 'h', [], 'files', {{}}, 'scene', [], 'name', '');
switch P.cam.source
    case 'webcam'
        if ~exist('webcamlist', 'file')
            error('camera_open:noSupportPackage', ['webcamlist not found. Install "MATLAB Support Package ' ...
                  'for USB Webcams" (Home -> Add-Ons -> Get Hardware Support Packages).']);
        end
        names = webcamlist;
        if isempty(names), error('camera_open:none', 'No camera found. Is the OV9281 plugged in?'); end
        idx = 1;
        if ~isempty(P.cam.deviceName)
            idx = find(contains(names, P.cam.deviceName, 'IgnoreCase', true), 1);
            if isempty(idx)
                error('camera_open:name', 'No camera matching "%s". Found: %s', P.cam.deviceName, strjoin(names, ', '));
            end
        end
        cam.name = names{idx};
        cam.h = webcam(idx);
        want = sprintf('%dx%d', P.cam.W, P.cam.H);
        if any(strcmp(cam.h.AvailableResolutions, want))
            cam.h.Resolution = want;
        else
            warning('camera_open:res', '%s not offered by %s, using %s', want, cam.name, cam.h.Resolution);
        end
        r = sscanf(cam.h.Resolution, '%dx%d'); cam.W = r(1); cam.H = r(2);
        snapshot(cam.h);                                   % first frame is often slow
    case 'video'
        if ~exist(P.cam.videoFile, 'file'), error('camera_open:file', 'Video file not found: %s', P.cam.videoFile); end
        cam.h = VideoReader(P.cam.videoFile);
        cam.W = cam.h.Width; cam.H = cam.h.Height; cam.name = P.cam.videoFile;
    case 'images'
        d = [dir(fullfile(P.cam.imageDir, '*.png')); dir(fullfile(P.cam.imageDir, '*.jpg'))];
        if isempty(d), error('camera_open:images', 'No PNG/JPG in %s', P.cam.imageDir); end
        cam.files = sort(fullfile(P.cam.imageDir, {d.name}));
        cam.name = P.cam.imageDir;
    case 'synthetic'
        if nargin < 2 || isempty(scene), scene = synth_scene(P, 'demo'); end
        cam.scene = scene; cam.name = ['synthetic (' scene.mode ')'];
    otherwise
        error('camera_open:source', 'Unknown P.cam.source "%s"', P.cam.source);
end
if cam.W ~= P.cam.W || cam.H ~= P.cam.H
    warning('camera_open:size', 'Frame size %dx%d differs from P.cam (%dx%d): f_px may be wrong.', ...
            cam.W, cam.H, P.cam.W, P.cam.H);
end
end
