function [H, zF] = face_head(box, H, zF, xCar, P)
%FACE_HEAD  Head position from a face box (P.trk.source = 'face').
%   box : [x y w h] from face_find ([] = no face)
%   H   : head struct from head_estimate (eyes/roll are kept; pos, z, valid replaced)
%   zF  : filtered distance [m] (NaN before the first face)
%   Distance from the box width: z = f * P.face.width / w (pinhole camera).
%   Position: the point at eye level in the middle of the box.
f = P.cam.f_px;
H.valid = false; H.pos = nan(3, 1);
if isempty(box)
    if ~isnan(zF), H.z = zF; end
    return
end
zNew = min(max(f*P.face.width/box(3), 0.2), 1.5);
if isnan(zF), zF = zNew; else, zF = (1 - P.face.zAlpha)*zF + P.face.zAlpha*zNew; end
u = box(1) - 0.5 + box(3)/2;                        % pixel-corner coordinates (cam_project)
v = box(2) - 0.5 + P.face.eyeLevel*box(4);
C = P.geo.cam + [xCar; 0; 0];
H.pos = C + [(u - P.cam.W/2)*zF/f; -(v - P.cam.H/2)*zF/f; zF];
H.z = zF;
H.valid = true;
end
