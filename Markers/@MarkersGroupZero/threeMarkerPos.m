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
%   Note that the arguments above are covectors.
%   SO3: 3 x 3, SO(3) rotation matrix
%   p: 3 x 1, position vector
%   @MarkersGroupZero

if nargin < 3
    % Compute the absolute position and orientation of the query markers
    originMarkers = [1,0,0; 0,0,0; 0,1,0];
end

% Definition

origin = originMarkers(2,:);
originRightOri = originMarkers(1,:) - origin;
originLeftOri = originMarkers(3,:) - origin;

query = queryMarkers(2,:);
queryRightOri = queryMarkers(1,:) - query;
queryLeftOri = queryMarkers(3,:) - query;

% Z-axis (Right-handed)

originUpOri = cross(originRightOri,originLeftOri);
queryUpOri = cross(queryRightOri,queryLeftOri);

% Frames (Unitized)

originZ = originUpOri/norm(originUpOri);
originX = originRightOri/norm(originRightOri);
originY = cross(originZ,originX);
queryZ = queryUpOri/norm(queryUpOri);
queryX = queryRightOri/norm(queryRightOri);
queryY = cross(queryZ,queryX);

% Position

p = query - origin;

% Orientation

rOrigin = [originX',originY',originZ'];
rQuery = [queryX',queryY',queryZ'];
SO3 = rQuery/rOrigin;

end

