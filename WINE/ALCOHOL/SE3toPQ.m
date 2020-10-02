function [DataOut] = SE3toPQ(SE3In)
%SE3toPQ Transform SE(3) data to position-quaternion form
%   Robotics systems toolbox is required
%   SE3In: 4 x 4 x N, data in
%   DataOut: N x 7, [x y z w, qx, qy, qz]

N = size(SE3In,3);
DataOut = zeros(N,7);
DataOut(:,1:3) = permute( SE3In(1:3,4,:), [3,1,2] );
DataOut(:,4:7) = tform2quat(SE3In);

end

