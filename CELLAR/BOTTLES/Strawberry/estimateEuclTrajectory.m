function [euclPosition,euclSO3] = estimateEuclTrajectory(markers)
%estimateEuclTrajectory Estimate the Euclidean trajectory of the rigid body
%given some markers. We always assume that the data length of the given
%markers are the same.
%   markers:    1 x M cell, N x 3 position data
%   -------------------------------------------------
%   euclPosition: N x 3, estimated position data
%   euclSO3: 3 x 3 x N, esitmated orientation data
%
%   Haopeng Hu
%   2020.09.29
%

N = size(markers{1},1);
euclPosition = zeros(N, 3);
euclSO3 = zeros(3, 3, N);

tmpMarker1 = markers{1};
tmpMarker2 = markers{2};
tmpMarker3 = markers{3};
tmpMarker4 = markers{4};
for i = 1:N
    [euclPosition(i,:), euclSO3(:,:,i)] = OMCSInter0929(tmpMarker1(i,:), tmpMarker2(i,:), tmpMarker3(i,:), tmpMarker4(i,:));
end

%% Interface version 09.29
% check: []
% x: 2 -> 4, y: 3, z: 1
    function [euclO, euclAxis] = OMCSInter0929(marker1, marker2, marker3, marker4)
        %OMCSInter0929 Used for interface 09.29
        %   markeri: 1 x 3, x y z
        %   --------------------------------------------
        %   euclO: 1 x 3, x y z of the estimated origin
        %   euclAxis: 3 x 3, x y z vector of the estimated frame
        tmpVec24 = (marker4 - marker2)'; 
        axisX = tmpVec24/norm(tmpVec24);
        tmpVec21 = (marker1 - marker2)';
        euclO = marker2' + (tmpVec21' * tmpVec24)/norm(tmpVec24) * axisX;
        tmpVec03 = marker3' - euclO;
        tmpVec04 = marker4' - euclO;
        axisZ = cross(tmpVec04,tmpVec03);   axisZ = axisZ/norm(axisZ);
        axisY = cross(axisZ, axisX);
        euclAxis = [axisX, axisY, axisZ];
        euclO = euclO'; % 3 x 1 -> 1 x 3
    end

end

