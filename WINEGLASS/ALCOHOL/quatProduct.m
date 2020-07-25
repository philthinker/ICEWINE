function [q] = quatProduct(q1,q2)
%quatProduct Quaternion product q1*q2
%   q1: 4 x N, quat
%   q2: 4 x N, quat
%   q: 4 x N, quat

N = size(q1,2);
q = repmat([1;0;0;0],[1,N]);
for i = 1:N
    tmpq = q1(:,i);
    q1Matrix = [ tmpq(1),  -tmpq(2),  -tmpq(3),  -tmpq(4);...
                        tmpq(2),   tmpq(1),  -tmpq(4),    tmpq(3);...
                        tmpq(3),   tmpq(4),    tmpq(1),  -tmpq(2);...
                        tmpq(4),  -tmpq(3),    tmpq(2),   tmpq(1) ];
    q(:,i) = q1Matrix*q2(:,i);
end

end

