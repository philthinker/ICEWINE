function [limits] = XYZLimits(X,Y,Z,LL)
%XYZLimits Return the x, y, z limits
%   X: N x 1,
%   Y: N x 1,
%   Z: N x 1,
%   LL: scalar, the length of axes (default:500)
%   limits: 1 x 6, limits used in axis()

if nargin < 4
    LL = 500;
end

Data = [X,Y,Z];
L = zeros(1,3);
limits = zeros(1,6);
for i = 1:3
    L(i) = max(Data(:,i)) - min(Data(:,i));
end
% Which one is the longest?
[~,maxI] = max(L);
for i = 1:3
    if i == maxI
        % The longest direction
        % We assume it is no more than LL.
        minValue = min(Data(:,i)) - 10;
        limits(1,i*2-1:i*2) = [minValue, minValue+LL];
    else
        % Other direction
        % They should be LL, too.
        meanValue = mean(Data(:,i));
        limits(1,i*2-1:i*2) = [meanValue-LL/2, meanValue+LL/2];
    end
end

end

