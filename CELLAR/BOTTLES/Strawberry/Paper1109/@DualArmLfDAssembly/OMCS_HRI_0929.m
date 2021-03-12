function [euclO, euclAxis] = OMCS_HRI_0929(obj, marker1, marker2, marker3, marker4)
%OMCS_HRI_0929 OMCS HRI for holder 0929: x: 2 -> 4, y: 3, z: 1
%   Note that here we DO NOT use the data of marker1
%   markeri: 1 x 3, x y z
%   --------------------------------------------
%   euclO: 3 x 1, x y z of the estimated origin
%   euclAxis: 3 x 3, x y z vector of the estimated frame
%
%   @DualArmLfDAssembly

% Calculate x
tmpVec24 = (marker4 - marker2)';
axisX = tmpVec24/norm(tmpVec24);
% Calculate origin
tmpVec23 = (marker3 - marker2)';
euclO = marker2' + (tmpVec23' * tmpVec24)/norm(tmpVec24) * axisX;
% Calculate y
tmpVec03 = marker3' - euclO;
axisY = tmpVec03/norm(tmpVec03);
% Calculate z
tmpVec04 = marker4' - euclO;
axisZ = cross(tmpVec04,tmpVec03);   axisZ = axisZ/norm(axisZ);

euclAxis = [axisX, axisY, axisZ];
% euclO = euclO'; % 3 x 1 -> 1 x 3

end

