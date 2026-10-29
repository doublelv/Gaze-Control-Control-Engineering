function pMeas = camera_measure(P, t, pEye)
%CAMERA_MEASURE  What the OV9281 + eye_detect deliver to MATLAB (simulation).
%   - new frame every 1/fps, the loop always uses the newest one
%   - latency P.cam.latency (exposure + MJPG + USB + decode)
%   - pupil-centre noise sigmaPx [px] -> metres via z/f
%   - eye distance z estimated with relative error zRelErr (from eye spacing)
%   - blinks: 150 ms without a measurement (NaN)
%   The carriage position is known from the encoder, so the camera only has
%   to measure the eye OFFSET from the camera; that is what the noise acts on.
%   pMeas : 3 x n, NaN where no measurement is available.
n = numel(t); dt = t(2) - t(1);
f = P.cam.f_px; s = P.cam.sigmaPx;
pMeas = nan(3, n);
zBias = 1 + P.cam.zRelErr;                       % constant distance-estimate error
lastFrame = -inf; held = nan(3,1);
for k = 1:n
    tc = t(k) - P.cam.latency;                   % time the newest frame was taken
    if tc < 0, continue, end
    frameT = floor(tc*P.cam.fps)/P.cam.fps;
    if frameT > lastFrame
        lastFrame = frameT;
        kf = max(1, round(frameT/dt) + 1);
        e = pEye(:,kf);
        if any(abs(frameT - P.cam.blinks) < 0.075 | ...
               (frameT >= P.cam.blinks & frameT < P.cam.blinks + 0.15))
            held = nan(3,1);                     % eye closed: pupil not found
        else
            z = e(3)*zBias;
            held = [e(1) + s*randn*z/f;          % lateral: pixel noise scaled by z/f
                    e(2) + s*randn*z/f;
                    z];
        end
    end
    pMeas(:,k) = held;
end
end
