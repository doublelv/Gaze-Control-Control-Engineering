function io = rig_open(mode, P)
%RIG_OPEN  Open the link to the Nucleo and check the firmware version.
%   mode = 'serial'  : real hardware over the ST-LINK virtual COM port (P.link.port)
%   mode = 'virtual' : simulated Nucleo + motor/belt that speaks the same byte
%                      protocol (no hardware, for testing the whole MATLAB side)
%   Errors with a clear message if the port does not exist or the firmware
%   does not answer HELLO with the expected protocol version.
%   Opening the link does NOT move any actuator.
C = proto_const();
io = struct('mode', mode, 'P', P, 'seq', uint8(0), 'nErr', 0, 'nTimeout', 0, ...
            'nWdog', 0, 'last', [], 'hello', [], 's', [], 'sim', []);
switch mode
    case 'serial'
        ports = string(serialportlist("available"));
        if ~any(ports == string(P.link.port))
            error('rig_open:noPort', 'Port %s not found. Available ports: %s\nSet P.link.port in params.m or config/local/params_local.m', ...
                  string(P.link.port), strjoin(ports, ', '));
        end
        io.s = serialport(P.link.port, P.link.baud, 'Timeout', P.link.timeout);
        pause(0.05); flush(io.s);
    case 'virtual'
        io.sim = virtual_mcu_init(P);
    otherwise
        error('rig_open:mode', 'unknown mode %s', mode);
end
% handshake
for attempt = 1:3
    io.seq = uint8(mod(double(io.seq) + 1, 256));
    [io, raw] = transact(io, encode_simple(C.T_HELLO, io.seq));
    h = decode_hello(raw, io.seq);
    if h.ok, break, end
end
if ~h.ok
    if strcmp(mode, 'serial'), delete(io.s); end
    error('rig_open:noHello', 'No HELLO reply from the Nucleo on %s. Is the firmware flashed and the baud rate %d?', ...
          string(P.link.port), P.link.baud);
end
if h.proto ~= C.version
    if strcmp(mode, 'serial'), delete(io.s); end
    error('rig_open:version', 'Protocol version mismatch: firmware %d, MATLAB %d', h.proto, C.version);
end
io.hello = h;
fprintf('Link OK (%s): protocol v%d, firmware %d.%d\n', mode, h.proto, h.fwMajor, h.fwMinor);
end
