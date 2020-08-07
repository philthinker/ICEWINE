function [qOut] = quatNormalize(qIn)
%quatNormalize Normalize the quat to unit quat
%   qIn: 4 x N, quat, [w x y z]'
%   qOut: 4 x N, quat, [w x y z]' 

N = size(qIn,2);
qOut = zeros(4,N);
for i = 1:N
    qOut(:,i) = qIn(:,i)/norm(qIn(:,i));
end

end

