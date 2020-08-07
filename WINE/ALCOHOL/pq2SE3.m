function [SE3] = pq2SE3(p,q)
%pq2SE3 Transform position and quaternion to the SE(3) form
%   p: 3 x N, position
%   q: 4 x N, quaternion
%   SE3: 4 x 4 x N, pose

N = min([size(p,2),size(q,2)]);
SE3 = repmat(eye(4),[1,1,N]);

SE3(1:3,4,:) = permute(p,[1,3,2]);
SO3 = quat2rotm(q');
SE3(1:3,1:3,:) = SO3;

end

