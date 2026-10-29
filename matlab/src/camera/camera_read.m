function [cam, img, info] = camera_read(cam, P, ctx)
%CAMERA_READ  Grab the next frame as a mono uint8 image.
%   ctx (synthetic source only): .t scene time [s], .xCar [m], .led (1xN logical),
%       .tilt (1x2 [rad])
%   info.ok    : false if no frame (end of video, camera error)
%   info.t     : time since camera_open [s] when the frame was received
%   info.idx   : frame counter,  info.truth : ground truth (synthetic only)
%   info.err   : error message when ok = false
info = struct('ok', false, 't', toc(cam.t0), 'idx', cam.nRead + 1, 'truth', [], 'err', '');
img = [];
try
    switch cam.source
        case 'webcam'
            img = snapshot(cam.h);
        case 'video'
            if ~hasFrame(cam.h), info.err = 'end of video'; return, end
            img = readFrame(cam.h);
        case 'images'
            if cam.nRead >= numel(cam.files), info.err = 'end of image list'; return, end
            img = imread(cam.files{cam.nRead + 1});
        case 'synthetic'
            if nargin < 3, ctx = struct(); end
            if ~isfield(ctx, 't'),    ctx.t = info.t; end
            if ~isfield(ctx, 'xCar'), ctx.xCar = 0; end
            if ~isfield(ctx, 'led'),  ctx.led = true(1, P.led.n); end
            if ~isfield(ctx, 'tilt'), ctx.tilt = [0 0]; end
            s = cam.scene.at(ctx.t);
            [img, tr] = synth_render(P, s, ctx.xCar, ctx.led, ctx.tilt);
            tr.scene = s; info.truth = tr;
    end
catch err
    info.err = err.message; img = []; return
end
if size(img, 3) == 3, img = img(:,:,1); end      % mono sensor delivered as RGB (MJPG)
if ~isa(img, 'uint8'), img = uint8(img); end
if P.cam.flipX, img = img(:, end:-1:1); end
if P.cam.flipY, img = img(end:-1:1, :); end
cam.nRead = cam.nRead + 1;
info.ok = true;
info.t = toc(cam.t0);
end
