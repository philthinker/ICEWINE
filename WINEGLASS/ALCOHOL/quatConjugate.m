function [bar_q] = quatConjugate(q,mod)
%quatConjugate Conjugate of unit quaternion
%   q: 4 x N, quat, [qw qx qy qz]
%   mod: boolean, true for row quat (default:false)
%   bar_q: 4 x N, quat

if nargin < 2
    mod = false;
end

if mod
    % row quat: N x 4
    bar_q = q;
    bar_q(:,2:4) = -q(:,2:4);
else
    % column quat: 4 x N
    bar_q = q;
    bar_q(2:4,:) = -q(2:4,:);
end

end

