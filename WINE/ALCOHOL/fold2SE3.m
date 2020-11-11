function [SE3Out] = fold2SE3(DataIn,mode)
%fold2SE3 Flod the N x 16 data into SE(3) form
%   DataIn: N x 16, pose data
%   mode: integer, 0 for column-major and 1 for row-major data (default: 0,
%   column-major)
%   SE3Out: 4 x 4 x N, SE3 data

if nargin < 2
    mode = 0;
end

if size(DataIn,2) < 12
    SE3Out = eye(4);
    return
end

N = size(DataIn,1);
SE3Out = repmat(eye(4),[1,1,N]);

if mode == 1
    % row-major data
    SE3Out(1,:,:) = permute( DataIn(:,1:4), [3,2,1]);
    SE3Out(2,:,:) = permute( DataIn(:,5:8), [3,2,1]);
    SE3Out(3,:,:) = permute( DataIn(:,9:12), [3,2,1]);
else
    % column-major data
    SE3Out(:,1,:) = permute( DataIn(:,1:4), [2,3,1]);
    SE3Out(:,2,:) = permute( DataIn(:,5:8), [2,3,1]);
    SE3Out(:,3,:) = permute( DataIn(:,9:12), [2,3,1]);
    SE3Out(:,4,:) = permute( DataIn(:,13:16), [2,3,1]);
end

end

