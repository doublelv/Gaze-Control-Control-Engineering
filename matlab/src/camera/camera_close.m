function camera_close(cam)
%CAMERA_CLOSE  Release the camera / file. Safe to call more than once.
if isfield(cam, 'h') && ~isempty(cam.h)
    try, delete(cam.h); catch, end
end
end
