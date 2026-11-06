%SERVO_TEST  Find P.srv.zeroUs and P.srv.dir for the pan servos.
%   Step 1: type pulse widths [us] until the LONG ARM of the servo horn points
%           straight at you (the user), then press Enter on an empty line.
%   Step 2: the script turns the servo 300 us further; type L if the long arm
%           turned to YOUR left (MAX end), R if to your right (MIN end).
%   Prints the lines for config/local/params_local.m at the end.
%   Hardware effects: servos move; motor stays off (duty 0), LEDs off.
%   Needs the 5-6 V servo supply. Stop any time with Ctrl+C.

clear; close all;
root = setup_paths();
P = params();
MODE   = 'serial';                     % 'serial' (rig) | 'virtual' (dry run)
SERVOS = [2];                          % which servos to test: 1 = PC6, 2 = PC7

io = rig_open(MODE, P);
cleanupRig = onCleanup(@() rig_close(io));
s = P.srv;
us = s.zeroUs; dirOut = s.dir;
cmd = struct('duty', 0, 'servoUs', us, 'led', false(1, P.led.n));
[io, fb] = rig_exchange(io, cmd);
if ~fb.ok, error('servo_test:link', 'No reply from the Nucleo'); end
degPerUs = s.range/(s.pulseMax - s.pulseMin)*180/pi;

for j = SERVOS
    pins = {'PC6', 'PC7'};
    fprintf('\n=== Servo %d (%s) ===\n', j, pins{j});
    fprintf('STEP 1: make the LONG ARM of the horn point straight at you.\n');
    while true
        cmd.servoUs = us; [io, fb] = rig_exchange(io, cmd);
        a = input(sprintf('  now %d us. New pulse (e.g. %d or %d), or just Enter when it points at you: ', ...
                  us(j), us(j) - 100, us(j) + 100), 's');
        if isempty(strtrim(a)), break, end
        v = str2double(a);
        if isnan(v), fprintf('  please type a number like 1400\n'); continue, end
        us(j) = round(min(max(v, s.pulseMin), s.pulseMax));
    end
    zero = us(j);
    test = min(zero + 300, s.pulseMax);
    fprintf('STEP 2: turning servo %d to %d us (about %.0f deg). Watch the long arm.\n', j, test, (test - zero)*degPerUs);
    us(j) = test; cmd.servoUs = us; [io, fb] = rig_exchange(io, cmd);
    a = '';
    while ~any(strcmpi(a, {'L', 'R'}))
        a = strtrim(input('  Did it turn to YOUR left (type L) or YOUR right (type R)? ', 's'));
    end
    dirOut(j) = 1 - 2*strcmpi(a, 'R');               % +1: longer pulse -> toward +x (MAX = your left)
    us(j) = zero; cmd.servoUs = us; [io, fb] = rig_exchange(io, cmd);   % back to pointing at you
    side = '';
    while ~any(strcmpi(side, {'L', 'R'}))
        side = strtrim(input('  Is this servo on the leg at YOUR left (L) or YOUR right (R)? ', 's'));
    end
    if strcmpi(side, 'L'), sideTxt = 'left (MAX)'; else, sideTxt = 'right (MIN)'; end
    fprintf('  servo %d: zero %d us, dir %+d, on your %s\n', j, zero, dirOut(j), sideTxt);
end

fprintf('\nCopy these lines into matlab/config/local/params_local.m (above "end"):\n');
fprintf('P.srv.zeroUs = [%d %d];\n', us);
fprintf('P.srv.dir    = [%d %d];\n', dirOut);
fprintf('Also tell which leg each tested servo is on (the code assumes servo 1 = right, servo 2 = left).\n');