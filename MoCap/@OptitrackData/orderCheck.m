function [flag, order] = orderCheck(obj, markerDist, epsilon)
%orderCheck Check the order of markers.
%   markerDist: Nm x Nm, distances between markers. At least 1 x Nm distances
%   are required.
%   epsilon: scalar, the allowed error (mm)
%   -------------------------------------------------
%   flag: boolean, true for pass
%   order: 1 x Nm, suggested order
%   @OptitrackData

Nm = obj.Nm;
flag = false;
order = (1:Nm);
epsilon = min([1, epsilon(1,1)]);

if size(markerDist,2) < Nm-1
    flag = false;
    return;
end

for i = 1:Nm % Row
    for j = i:Nm % Column
        
    end
end

end

