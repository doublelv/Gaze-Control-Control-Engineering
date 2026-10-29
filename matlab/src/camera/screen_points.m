function [X, uvScreen] = screen_points(P, grid, margin)
%SCREEN_POINTS  Calibration / demo target positions on the screen.
%   X        : 3xN world positions [m] (screen plane z = 0)
%   uvScreen : 2xN positions in normalised screen coordinates (0..1, origin top-left)
%   Order: row by row from the top-left.
if nargin < 2, grid = P.gcal.grid; end
if nargin < 3, margin = P.gcal.margin; end
nx = grid(1); ny = grid(2);
gx = linspace(margin, 1 - margin, nx);
gy = linspace(margin, 1 - margin, ny);
[U, V] = meshgrid(gx, gy);
U = U'; V = V';                                % row by row
uvScreen = [U(:)'; V(:)'];
X = [P.screen.centre(1) + (uvScreen(1,:) - 0.5)*P.screen.size(1);
     P.screen.centre(2) - (uvScreen(2,:) - 0.5)*P.screen.size(2);
     P.screen.centre(3) + zeros(1, nx*ny)];
end
