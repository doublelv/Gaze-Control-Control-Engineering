function L = led_world_pos(P, xCar)
%LED_WORLD_POS  World positions of all IR LEDs [m], 3 x P.led.n.
%   xCar : carriage (camera) position [m] (scalar). Only the carriage LEDs
%          depend on it; the frame LEDs (P.led.onFrame) are fixed.
%   Frame LED x in P.led.pos is measured from the camera lens with the carriage
%   at the MIN stop (P.belt.xStop), so world x = xStop + pos(1).
if nargin < 2 || isempty(xCar), xCar = 0; end
C = P.geo.cam + [xCar; 0; 0];
L = C + P.led.pos;
fr = logical(P.led.onFrame);
if any(fr)
    L(:, fr) = P.geo.cam + [P.belt.xStop; 0; 0] + P.led.pos(:, fr);
end
end
