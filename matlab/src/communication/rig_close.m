function rig_close(io)
%RIG_CLOSE  Send STOP (motor 0, LEDs off) and release the port. Safe to call twice.
try
    rig_stop(io);
catch
end
if strcmp(io.mode, 'serial') && ~isempty(io.s)
    try, delete(io.s); catch, end
end
end
