function [eta] = quatLogMap(q,qa)
%quatLogMap Logrithmic map of unit quaternion
%   q: 4 x N, quat, w x y z
%   qa: 4 x N, quat, w x y z (default: [1,0,0,0]')

if nargin < 2
    qa = [1,0,0,0]';
end

N = size(q,2);
eta = zeros(3,N);

if size(qa,2) < N
    qa = repmat(qa(:,1),[1,N]);
end
bar_qa = quatConjugate(qa);
dq = quatProduct(q,bar_qa);
disp(dq);

w = acos(dq(1,:));
for i =1:N
    if norm(dq(2:4,i)) < 1e-6
        eta(:,i) = [0 0 0]';
    else
        eta(:,i) = w(i)*dq(2:4,i)/norm(dq(2:4,i));
    end
end

end

