%RECORD_SESSION  Record camera frames for offline tuning (tune_detector, run_eye_demo).
%   Saves an AVI to matlab/data/local/ (git-ignored: recordings of eyes are
%   personal data, never commit them).
%   LEDS = true : the Nucleo keeps the LEDs in P.led.on switched on while
%                 recording (same light as run_rig track_camera); servos at
%                 P.srv.zeroUs, motor off (duty 0). Needs the rig on P.link.port.
%   LEDS = false: camera only, no hardware effects.
%   SMALL = true: Motion JPEG (about 20x smaller, small enough to upload);
%   SMALL = false: lossless grayscale (about 1 MB per frame).

clear; close all;
[root, mroot] = setup_paths();
P = params();
SECONDS = 10;
LEDS    = true;
SMALL   = true;
P.cam.source = 'webcam';

if LEDS
    io = rig_open('serial', P);
    cleanupRig = onCleanup(@() rig_close(io));
    cmd = struct('duty', 0, 'servoUs', P.srv.zeroUs, 'led', logical(P.led.on));
    [io, fb] = rig_exchange(io, cmd);
    if ~fb.ok, error('record_session:link', 'No reply from the Nucleo'); end
end
cam = camera_open(P);
cleanupCam = onCleanup(@() camera_close(cam));
d = fullfile(mroot, 'data', 'local'); if ~exist(d, 'dir'), mkdir(d); end
f = fullfile(d, sprintf('session_%s.avi', datestr(now, 'yyyymmdd_HHMMSS')));
if SMALL
    vw = VideoWriter(f, 'Motion JPEG AVI'); vw.Quality = 90;
else
    try
        vw = VideoWriter(f, 'Grayscale AVI');
    catch
        vw = VideoWriter(f, 'Motion JPEG AVI'); vw.Quality = 100;
    end
end
vw.FrameRate = 30;
open(vw);
fig = figure('Name', 'Recording - close to stop'); ax = axes(fig); hI = [];
t0 = tic; n = 0;
while toc(t0) < SECONDS && viz_alive(fig)
    if LEDS, [io, fb] = rig_exchange(io, cmd); end      % keeps the LEDs on (100 ms watchdog)
    [cam, img, inf_] = camera_read(cam, P);
    if ~inf_.ok, break, end
    writeVideo(vw, img); n = n + 1;
    if mod(n, 5) == 1
        if isempty(hI), hI = imshow(img, 'Parent', ax); else, set(hI, 'CData', img); end
        title(ax, sprintf('%.1f s, %d frames', toc(t0), n)); viz_drawnow();
    end
end
close(vw);
fi = dir(f);
fprintf('%d frames (%.1f fps), %.1f MB -> %s\n', n, n/toc(t0), fi.bytes/1e6, f);
if LEDS, rig_close(io); end
