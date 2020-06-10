function [SE3Out] = fold2SE3(DataIn)
%fold2SE3 Flod the N x 16 data into SE(3) form
%   We always omit the last 4 entries in each row when size(DataIn,2) == 16
%   DataIn: N x 16 or N x 12, pose data

if size(DataIn,2) < 12
    SE3Out = eye(4);
    return
end

N = size(DataIn,1);
SE3Out = repmat(eye(4),[1,1,N]);

SE3Out(1,:,:) = permute( DataIn(:,1:4), [3,2,1]);
SE3Out(2,:,:) = permute( DataIn(:,5:8), [3,2,1]);
SE3Out(3,:,:) = permute( DataIn(:,9:12), [3,2,1]);

end

