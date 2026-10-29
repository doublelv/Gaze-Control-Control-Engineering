function test_link_virtual()
%TEST_LINK_VIRTUAL  Whole MATLAB link layer against the virtual MCU:
%   handshake, CMD/STATE round trip, endstop gating, watchdog, corrupt frames.
P = params(); rng(4);
io = rig_open('virtual', P);
assert(io.hello.ok && io.hello.proto == proto_const().version, 'handshake');
cmd = struct('duty', -0.3*P.drv.pwmCounts, 'servoUs', P.srv.zeroUs, 'led', false(1, 4));
hit = false;
for k = 1:1000                                   % drive into the MIN endstop
    [io, fb] = rig_exchange(io, cmd);
    assert(fb.ok, 'reply missing');
    if fb.endMin, hit = true; break, end
end
assert(hit, 'endstop never reached');
c0 = fb.count;
for k = 1:20, [io, fb] = rig_exchange(io, cmd); end
assert(fb.limited && abs(double(fb.count - c0)) < 50, 'endstop does not stop the motor');
% watchdog: no command for 0.2 s while driving -> motor stopped, flag set
cmd.duty = 0.3*P.drv.pwmCounts;
[io, fb] = rig_exchange(io, cmd);
for k = 1:20, io.sim = virtual_mcu_step(io.sim, uint8([1 2 3]), P); end   % garbage only
[io, fb] = rig_exchange(io, cmd);
assert(fb.wdog && fb.rxErr && fb.rxErrors >= 20, 'watchdog / rx error flags');
rig_close(io);
end
