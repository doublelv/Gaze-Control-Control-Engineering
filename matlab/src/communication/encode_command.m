function pkt = encode_command(cmd, seq, P)
%ENCODE_COMMAND  CMD frame (PC -> MCU), payload 8 bytes:
%   int16 duty | uint16 servoA_us | uint16 servoB_us | uint8 ledMask | uint8 seq
%   cmd.duty    : PWM counts, clipped to [-pwmCounts, pwmCounts] (sign = direction)
%   cmd.servoUs : [A B] pulse widths [us], clipped to [pulseMin, pulseMax]
%   cmd.led     : 1 x nLED logical, LED i -> bit i-1
%   P.led.twin  : optional, twin(i) = k > 0 -> LED k (bit k-1) switches together
%                 with LED i (second LED on the same servo)
C = proto_const();
on = logical(cmd.led(:)');
if isfield(P.led, 'twin')
    for i = find(P.led.twin(1:numel(on)) > 0 & on), on(P.led.twin(i)) = true; end
end
duty = int16(max(min(round(cmd.duty), P.drv.pwmCounts), -P.drv.pwmCounts));
sv   = uint16(min(max(round(cmd.servoUs), P.srv.pulseMin), P.srv.pulseMax));
led  = uint8(sum(double(on) .* 2.^(0:numel(on)-1)));
payload = [typecast(duty, 'uint8'), typecast(sv(1), 'uint8'), typecast(sv(2), 'uint8'), led, uint8(seq)];
pkt = proto_frame(C.T_CMD, payload);
end
