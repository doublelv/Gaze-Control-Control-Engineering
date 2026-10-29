function viz_drawnow()
%VIZ_DRAWNOW  drawnow limitrate in MATLAB, plain drawnow where not supported (Octave).
persistent useLimit
if isempty(useLimit), useLimit = ~exist('OCTAVE_VERSION', 'builtin'); end
   if useLimit, drawnow limitrate; else, drawnow; end
end
