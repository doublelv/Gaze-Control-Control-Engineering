function [hold, holding] = hold_band(holding, steady, errM, P)
%HOLD_BAND  Motor-off zone around a still target, with hysteresis.
%   holding : state from the previous loop (start with false)
%   steady  : the (slew-limited) reference did not change this loop
%   errM    : |reference - carriage| [m]
%   hold    : true -> switch the motor off this loop (the PI state is kept)
%   The motor is switched off only once the error is below P.trk.holdBand
%   (small: the integrator first drives the error to ~0, so there is no
%   steady-state error) and on again when it grows above P.trk.holdExit
%   (no hunting around the target because of the friction).
%   Older logs have no P.trk.holdExit: then exit = enter (the old behaviour).
if isfield(P.trk, 'holdExit'), hExit = P.trk.holdExit; else, hExit = P.trk.holdBand; end
if ~steady
    holding = false;
elseif holding
    holding = errM < hExit;
else
    holding = errM < P.trk.holdBand;
end
hold = holding;
end
