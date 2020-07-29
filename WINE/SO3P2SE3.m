function [SE3] = SO3P2SE3(SO3,p)
%SO3P2SE3 Transform SO3 and postion to SE3
%   SO3: 3 x 3 x N, rotational transformation matrix
%   p: 3 x N, position (optional)
%   SE3: 4 x 4 x N, homogeneous transformation matrix

N = size(SO3,3);

if nargin < 2
    p = repmat([0,0,0]',[1,N]);
end

SE3 = repmat(eye(4),[1,1,N]);
SE3(1:3,4,:) = permute(p,[1,3,2]);
SE3(1:3,1:3,:) = SO3;

end

