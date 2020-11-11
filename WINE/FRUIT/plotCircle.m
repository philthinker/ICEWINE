function [circle] = plotCircle(center,r,c,s,N)
%plotCircle Plot a 2-D circle
%   center: 1 x 2, Center
%   r:  scalar, Radius
%   c:  1x 3, Color
%   s:  scaler, LineWidth
%   N: integer, the num. of circle data (default: 200)
%   -------------------------------------------------
%   circle: N x 2, the circle data

if nargin < 5
    N = 200;
end

circle = repmat(center,[N,1]);
circle(:,1) = center(1) + r * cos(linspace(0,2*pi, N));
circle(:,2) = center(2) + r * sin(linspace(0,2*pi, N));

plot(circle(:,1), circle(:,2), 'Color', c, 'LineWidth', s);

end

