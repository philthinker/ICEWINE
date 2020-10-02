function [passFlag] = checkMarkersOrder(markers,distances,epsilon)
%checkMarkersOrder Check the order of the markers' data. We check the
%distances between marker 1 and 2, 2 and 3, ... M and 1
%   markers: 1 x M cell, N x 3 markers' position data
%   distances: 1 x M, pre-defined distances
%   epsilon: scaler, the allowable error
%   -------------------------------------------------
%   passFlag: boolean, true for pass

passFlag = true;
M = size(markers,2);
if M <= 1
    passFlag = true;
    return;
end

for i = 2:M+1
    if i == M + 1
        tmpData = markers{M}-markers{1};
    else
        tmpData = markers{i}-markers{i-1};
    end
    tmpDist = sqrt(diag(tmpData * (tmpData')));
    tmpErr = abs(tmpDist - distances(i-1)) > epsilon;
    errRate = sum(tmpErr)/size(tmpDist,1);
    passFlag = passFlag && (errRate < 0.05);
end


end

