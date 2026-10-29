%RECORD_SESSION  Record camera frames for offline tuning (tune_detector, run_eye_demo).
%   Saves an AVI (lossless grayscale if supported) to matlab/data/local/
%   (git-ignored: recordings of eyes are personal data, never commit them).
%   Hardware effects: none.

clear; close all;
[root, mroot] = setup_paths();
P = params();
SECONDS = 20;
P.cam.source = 'webcam';
cam = camera_open(P);
cleanupCam = onCleanup(@() camera_close(cam));
d = fullfile(mroot, 'data', 'local'); if ~exist(d, 'dir'), mkdir(d); end
f = fullfile(d, sprintf('session_%s.avi', datestr(now, 'yyyymmdd_HHMMSS')));
try
    vw = VideoWriter(f, 'Grayscale AVI');
catch
    vw = VideoWriter(f, 'Motion JPEG AVI'); vw.Quality = 100;
end
vw.FrameRate = 30;
open(vw);
fig = figure('Name', 'Recording - close to stop'); ax = axes(fig); hI = [];
t0 = tic; n = 0;
while toc(t0) < SECONDS && viz_alive(fig)
    [cam, img, inf_] = camera_read(cam, P);
    if ~inf_.ok, break, end
    writeVideo(vw, img); n = n + 1;
    if mod(n, 5) == 1
        if isempty(hI), hI = imshow(img, 'Parent', ax); else, set(hI, 'CData', img); end
        title(ax, sprintf('%.1f s, %d frames', toc(t0), n)); viz_drawnow();
    end
end
close(vw);
fprintf('%d frames (%.1f fps) -> %s\n', n, n/toc(t0), f);
