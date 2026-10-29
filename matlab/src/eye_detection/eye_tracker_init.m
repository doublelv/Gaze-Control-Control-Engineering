function T = eye_tracker_init(P)
%EYE_TRACKER_INIT  State of the two-eye tracker.
%   T.status : 'search' (no lock, full-frame search, needs an eye PAIR to lock)
%              'track'  (at least one eye seen recently, ROI search)
%              'blink'  (both eyes missing for < P.trk.blinkMax, ROI search)
%              'lost'   (missing > P.trk.lostAfter -> back to full-frame search)
%   T.uv     : 2x2 last pupil positions [right left] (full px)
%   T.vel    : 2x2 pupil velocities [px/s]
%   T.offset : 2x1 right - left pupil vector of the last seen pair [px]
T.status = 'search';
T.uv = nan(2, 2);
T.vel = zeros(2, 2);
T.offset = [NaN; NaN];
T.tSeen = [-inf -inf];
T.tLast = NaN;
T.z = P.head.zDefault;
T.nFrames = 0;
T.nLocks = 0;
end
