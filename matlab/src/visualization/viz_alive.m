function ok = viz_alive(h)
%VIZ_ALIVE  True if the graphics handle h still exists (isvalid is not in Octave).
ok = ~isempty(h) && ishghandle(h);
end
