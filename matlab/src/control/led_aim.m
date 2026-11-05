function [ang, beam] = led_aim(P, Lpos, pTarget)
%LED_AIM  Servo angle that points an LED at a target, and the beam direction.
%   Lpos    : 3x1 LED position [m] (world)
%   pTarget : 3 x n target positions [m] (world), e.g. the eye
%   ang     : 1 x n servo angle [rad], 0 = straight ahead (+z)
%             P.led.aim 'pan' : theta = atan2(x_t - x_L, z_t - z_L), + toward +x
%             P.led.aim 'tilt': theta = atan2(y_t - y_L, hypot(dx, dz)), + up
%   beam    : function handle a -> 3x1 unit beam direction for servo angle a
switch P.led.aim
    case 'pan'
        ang  = atan2(pTarget(1,:) - Lpos(1), pTarget(3,:) - Lpos(3));
        beam = @(a) [sin(a); 0; cos(a)];
    case 'tilt'
        ang  = atan2(pTarget(2,:) - Lpos(2), hypot(pTarget(1,:) - Lpos(1), pTarget(3,:) - Lpos(3)));
        beam = @(a) [0; sin(a); cos(a)];
    otherwise
        error('led_aim:aim', 'P.led.aim must be ''pan'' or ''tilt'', not ''%s''', P.led.aim);
end
end
