function [qOut] = quatRegulate(qIn,mod)
%quatRegulate Regulate the unit quaternion to positive scalar mod
%   qIn: 4 x N, quat, [qw qx qy qz]
%   mod: boolean, true for row quat (default:false)
%   -------------------------------------------------
%   qOut: 4 x N, quat

if nargin < 2
    mod = false;
end

qOut = qIn;
if mod
    % row quat
    indices = qIn(:,1) < 0;
    qOut(indices,:) = -qIn(indices,:);
else
    % column quat
    indices = qIn(1,:) < 0;
    qOut(:,indices) = -qIn(:,indices);
end

end

