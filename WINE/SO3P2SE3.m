function [SE3] = SO3P2SE3(SO3,p)
%SO3P2SE3 Transform SO3 and postion to SE3
%   SO3: 3 x 3, rotational transformation matrix
%   p: 3 x 1, position (optional)
%   SE3: 4 x 4, homogeneous transformation matrix

if nargin < 2
    p = [0,0,0]';
end
SE3 = eye(4);
SE3(1:3,4) = p;
SE3(1:3,1:3) = SO3;

end

