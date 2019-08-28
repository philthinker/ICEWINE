function [SO3,p] = threeMarkerPos(obj,queryMarkers,originMarkers)
%threeMarkerPos Compute the relative posture of the query markers according
%to the origin markers. At least three markers are required.
%   %%%%------------------------------------------------------%%%%
%   By default, the middle marker is taken as the origin and the center of the left two
%   markers is used to indicate the orientation. This can be modified for
%   future use.
%   %%%%------------------------------------------------------%%%%
%   queryMarkers: 3 x 3, xyz of the query markers
%   originMarkers: 3 x 3, xyz of the origin markers (optional)
%   SO3: 3 x 3, SO(3) rotation matrix
%   p: 3 x 1, position vector
%   @MarkersGroupZero

if nargin < 3
    % Compute the absolute position and orientation of the query markers
    originMarkers = [1,0,0; 0,0,0; 0,1,0];
end

%%%%--------------------------------------------------%%%%
% Modify the following codes if you want other origin defination
origin = originMarkers(2,:);
originOri = (originMarkers(1,:) + originMarkers(3,:))/2 - origin;
originOriLeft = originMarkers(1,:) - origin;
originOriRight = originMarkers(3,:) - origin;
query = queryMarkers(2,:);
queryOri = (queryMarkers(1,:) + queryMarkers(3,:))/2 - query;
queryOriLeft = queryMarkers(1,:) - query;
queryOriRight = queryMarkers(3,:) - query;
%%%%--------------------------------------------------%%%%


% Unitizing
originOri = originOri/norm(originOri);
queryOri = queryOri/norm(queryOri);

% We define the orientation vector as z-axis, and the vector perpendicular
% to the surface defined by marker 1 to 3 as y-axis.
% Then axis-x = axis-y x axis-z.
% You can modify these codes for your convenience.
originZ = originOri;
originY = cross(originOriRight,originOriLeft); originY = originY/norm(originY);
originX = cross(originY,originZ);
Ro = [originX, originY, originZ];
queryZ = queryOri;
queryY = cross(queryOriRight,queryOriLeft); queryY = queryY/norm(queryY);
queryX = cross(queryY, queryZ);
Rq = [queryX, queryY, queryZ];
Rqo = Rq/Ro;

tmpP = query - origin;

p = [dot(tmpP,originX),dot(tmpP,originY),dot(tmpP,originZ)];
SO3 = Rqo;

end

