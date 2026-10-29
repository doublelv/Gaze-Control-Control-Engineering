%MOTOR_TEST  Spin the carriage motor open-loop (forward, stop, backward, stop)
%   with all 4 IR LEDs switched on during the test.
%   First motor test on the real rig. Prints the encoder counts and speed.
%   !! TAKE THE BELT OFF THE PULLEY FIRST (or make sure the carriage is in the
%   middle of the rail): there is no position control and no homing here. !!
%   The firmware still stops the motor if the MIN endstop is pressed while
%   driving toward MIN, and 100 ms after MATLAB stops sending (watchdog).
%   Stop any time with Ctrl-C (the motor is stopped on exit).

clear; close all;
setup_paths();

MODE  = 'serial';   % 'serial' = real Nucleo on P.link.port | 'virtual' = no hardware
VOLTS = 3;          % motor voltage [V] (12 V supply); start low: 2-4 V
TRUN  = 2;          % seconds per direction

if ~isempty(getenv('RIG_MODE')), MODE = getenv('RIG_MODE'); end   % used by the tests

P = params();
M = build_axis_model(P.ax(1), P);
s = P.srv;
io = rig_open(MODE, P);
cleanupRig = onCleanup(@() rig_close(io));
realtime = strcmp(MODE, 'serial');
cmd = struct('duty', 0, 'servoUs', s.zeroUs, 'led', true(1, P.led.n));   % all LEDs ON
duty = round(min(abs(VOLTS), P.psu.V)/P.psu.V*P.drv.pwmCounts);

fprintf('\nMotor test: %.1f V (duty %d of %d), %g s each way. Ctrl-C = stop.\n', ...
        VOLTS, duty, P.drv.pwmCounts, TRUN);
if realtime
    input('Belt OFF (or carriage in the middle)? Enter = go, Ctrl-C = abort: ', 's');
end

phases = {'forward', +1; 'stop', 0; 'backward', -1; 'stop', 0};
for p = 1:size(phases, 1)
    name = phases{p, 1}; sgn = phases{p, 2};
    T = TRUN; if sgn == 0, T = 0.5; end
    cmd.duty = sgn*duty;
    tk = struct('t0', tic, 'rt', realtime, 'Ts', P.Ts);
    c0 = NaN; c1 = NaN; nOk = 0; nLim = 0;
    for k = 1:round(T/P.Ts)
        [io, fb] = rig_exchange(io, cmd);
        if fb.ok
            nOk = nOk + 1;
            if isnan(c0), c0 = double(fb.count); end
            c1 = double(fb.count);
            if isfield(fb, 'limited') && fb.limited, nLim = nLim + 1; end
        end
        rig_tick_wait(tk, k);
    end
    dc = c1 - c0;
    revs = dc*M.q/(2*pi);                        % output-shaft revolutions
    note = ''; if nLim > 0, note = sprintf(', endstop blocked %d', nLim); end
    fprintf('%-8s: %+7d counts = %+6.2f rev = %+6.1f mm belt, %5.1f rpm  (replies %d/%d%s)\n', ...
            name, dc, revs, revs*2*pi*M.r*1e3, revs/T*60, nOk, round(T/P.Ts), note);
end
cmd.duty = 0; cmd.led = false(1, P.led.n); rig_exchange(io, cmd);   % motor off, LEDs off
fprintf(['\nLEDs were ON during the test (850 nm: invisible to the eye, check with a phone camera).\n' ...
         'Check: forward should give POSITIVE counts (toward MAX).\n' ...
         'No movement -> check 12 V, MDD10A wiring (PWM1 = PB8/D15, DIR1 = PB9/D14, GND).\n' ...
         'Moves but counts stay 0 -> check encoder wiring (A = PA5/D13, B = PB3, 3V3, GND).\n' ...
         'Forward gives negative counts -> swap the motor wires (M1A/M1B).\n']);

