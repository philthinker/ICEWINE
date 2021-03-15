function [q] = quatExpMap(eta,qa)
%quatExpMap Exponential map of unit quaternion
%   eta: 3 x N, eta, x y z
%   qa: 4 x N, quat, w x y z (default: [1,0,0,0]')

if nargin < 2
    qa = [1,0,0,0]';
end

N = size(eta,2);
q = zeros(4,N);
tmpQ = q;

if size(qa,2) < N
    qa = repmat(qa(:,1),[1,N]);
end

for i = 1:N
    normEta = norm(eta(:,i));
    if normEta == 0
        tmpQ(:,i) = [1 0 0 0]';
    else
        tmpQ(:,i) = [cos(normEta); sin(normEta)*eta(:,i)/normEta];
    end
end

q = quatProduct(tmpQ,qa);

end

