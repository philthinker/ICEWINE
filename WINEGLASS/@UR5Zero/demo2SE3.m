function [demoCartesian] = demo2SE3(obj,demo)
%demo2SE3 Transform Cartesian demo data to its SE3 form
%   demo: N x 16, demo in Cartesian space
%   demoCartesian: 4 x 4 x N, demo in SE3 form
%   @UR5Zero

N = size(demo,1);
demoCartesian = repmat(eye(4,4),[1,1,N]);
for i = 1:4
    demoCartesian(:,i,:) = demo(:,(i-1)*4+1:i*4)';
end

end

