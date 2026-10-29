function F = gaze_features(obs, H, P)
%GAZE_FEATURES  Pupil-centre / corneal-reflection (PCCR) feature per eye.
%   F.v     : 2x2 [right left] PCCR vectors, pupil - glint centroid, converted
%             from pixels to metres at the eye (x z/f) so the feature depends
%             little on the viewing distance. NaN if the eye or its glint is missing.
%   F.valid : 1x2 logical,  F.nGlints : 1x2 glints used
%   F.z     : eye distance used [m]
%   With the 1.65 mm lens all LED glints merge into ~1 spot; the centroid of
%   whatever was found is used, so keep the same LEDs on as during calibration.
F = struct('v', nan(2, 2), 'valid', [false false], 'nGlints', [0 0], 'z', H.z);
for e = 1:2
    if ~obs.seen(e) || isempty(obs.eye{e}) || any(isnan(obs.eye{e}.pccr)), continue, end
    F.v(:, e) = obs.eye{e}.pccr(:)*H.z/P.cam.f_px;
    F.valid(e) = true;
    F.nGlints(e) = size(obs.eye{e}.glints, 1);
end
end
