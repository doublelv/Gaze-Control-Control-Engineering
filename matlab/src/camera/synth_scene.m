function S = synth_scene(P, mode, varargin)
%SYNTH_SCENE  Ground truth for the synthetic camera at time t.
%   S = synth_scene(P, 'demo')            moving head + roll + gaze hopping over a
%                                         3x3 screen grid + blinks (P.cam.blinks)
%   S = synth_scene(P, 'calib', target)   still head (small sway), fixed gaze target
%   S.at(t) returns a struct: head (3x1 eye midpoint), roll [rad], target (3x1),
%   blink (logical), eyes (3x2 rotation centres: [right left] in the image).
%   Units: metres, world frame of params.m (x right, y up, z toward the user).
S.mode = mode;
switch mode
    case 'demo'
        S.at = @(t) state(P, eye_trajectory(P, t), P.syn.rollAmp*pi/180*sin(2*pi*0.1*t), ...
                          demo_target(P, t), is_blink(P, t));
    case 'calib'
        tgt = varargin{1};
        S.at = @(t) state(P, P.traj.p0 + [0.004*sin(2*pi*0.2*t); 0.003*sin(2*pi*0.13*t); 0], ...
                          0, tgt, false);
    otherwise
        error('synth_scene: unknown mode %s', mode);
end
end

function s = state(P, head, roll, target, blink)
s.head = head; s.roll = roll; s.target = target; s.blink = blink;
Rr = [cos(roll) -sin(roll) 0; sin(roll) cos(roll) 0; 0 0 1];
half = Rr*[P.det.ipd/2; 0; 0];
s.eyes = [head + half, head - half];          % [image-right eye, image-left eye]
end

function b = is_blink(P, t)
b = any(t >= P.cam.blinks & t < P.cam.blinks + 0.15);
end

function tgt = demo_target(P, t)
pts = screen_points(P);
k = mod(floor(t/P.syn.gazeDwell), size(pts, 2)) + 1;
tgt = pts(:, k);
end
