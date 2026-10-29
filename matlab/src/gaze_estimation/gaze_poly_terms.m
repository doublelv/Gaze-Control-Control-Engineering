function A = gaze_poly_terms(v, order)
%GAZE_POLY_TERMS  Polynomial regressors of the PCCR feature v (2 x N).
%   order 1: [1 x y],  order 2: [1 x y xy x^2 y^2]   (N x 3 or N x 6)
x = v(1, :)'; y = v(2, :)';
switch order
    case 1, A = [ones(size(x)) x y];
    case 2, A = [ones(size(x)) x y x.*y x.^2 y.^2];
    otherwise, error('gaze_poly_terms: order must be 1 or 2');
end
end
