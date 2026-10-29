function C = proto_const()
%PROTO_CONST  Constants of the MATLAB <-> STM32 protocol (interface/protocol.md, v1).
C.version   = 1;
C.header    = uint8([170 85]);          % 0xAA 0x55
C.maxLen    = 16;                       % max payload bytes
% message types
C.T_CMD         = uint8(1);             % PC -> MCU: motor, servos, LEDs
C.T_HELLO       = uint8(2);             % PC -> MCU: version query
C.T_STOP        = uint8(3);             % PC -> MCU: motor 0, LEDs off, servos hold
C.T_STATE       = uint8(129);           % MCU -> PC: answer to CMD and STOP (0x81)
C.T_HELLO_REPLY = uint8(130);           % MCU -> PC: answer to HELLO (0x82)
% STATE.flags bits
C.F_END_MIN   = 1;    % bit0 MIN endstop pressed
C.F_END_MAX   = 2;    % bit1 MAX endstop pressed
C.F_WDOG      = 4;    % bit2 watchdog tripped since the previous STATE
C.F_LIMITED   = 8;    % bit3 duty forced to 0 by an endstop
C.F_RX_ERR    = 16;   % bit4 malformed frame seen since the previous STATE
end
