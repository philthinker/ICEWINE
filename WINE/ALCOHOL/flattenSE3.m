function [DataOut] = flattenSE3(DataIn, mode)
%flattenSE3 Flatten a SE(3) trajectory
%   DataIn: 4 x 4 x N, SE(3) data
%   mode: integer, 0 for column-major and 1 for row-major data (default: 0,
%   column-major)
%   DataOut: N x 16, flattened SE(3) data

if nargin < 2
    mode = 0;
end

if size(DataIn,1) ~= 4 || size(DataIn,2) ~= 4
    DataOut = flattenSE3(eye(4));
    return
end

N = size(DataIn,3);
DataOut = zeros(N,16);

if mode == 1
    % row-major data
    DataOut(:, 1:4) = permute(DataIn(1,:,:), [3,2,1]);
    DataOut(:, 5:8) = permute(DataIn(2,:,:), [3,2,1]);
    DataOut(:, 9:12) = permute(DataIn(3,:,:), [3,2,1]);
    DataOut(:, 13:16) = permute(DataIn(4,:,:), [3,2,1]);
else
    % column-major data
    DataOut(:, 1:4) = permute(DataIn(:,1,:), [3,1,2]);
    DataOut(:, 5:8) = permute(DataIn(:,2,:), [3,1,2]);
    DataOut(:, 9:12) = permute(DataIn(:,3,:), [3,1,2]);
    DataOut(:, 13:16) = permute(DataIn(:,4,:), [3,1,2]);
end

end

