function [X,U,V,W] = computeQuiver3(x,y,z,theta,axis,mode)
%computeVer3 Compute the ends of normal vectors for quiver3
%   We assume that the original orientation is the same as the world frame
%   x: scalar, x
%   y: scalar, y
%   z: scalar, z
%   theta: scalar, radial
%   axis: 1 x 3, rotation axis
%   mode: integer, computation mode (default:0)
%   X: 1 x 3, [x,y,z]'
%   U: 1 x 3, u
%   V: 1 x 3, v
%   W: 1 x 3, w

if nargin < 6
    mode = 0;
end

L = 1;  % Edit this factor to scale the length of axis

X = [x,y,z]';
U = zeros(1,3);
V = zeros(1,3);
W = zeros(1,3);

X0 = [1,0,0]';
Y0 = [0,1,0]';
Z0 = [0,0,1]';

if floor(abs(mode)) == 0
    % SO3 rotation
    R = axang2rotm([axis,theta]);
    U = R*X0.*L;
    V = R*Y0.*L;
    W = R*Z0.*L;
end

end

