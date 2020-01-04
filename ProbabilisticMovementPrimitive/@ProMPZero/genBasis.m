function [Phi] = genBasis(obj,z)
%genBasis Generate basis given phase z
%   z: 1 x N, phase variable from 0 to 1
%   Phi: N x K, basis
%   @ProMPZero

K = obj.nKernel;
N = length(z);
c = obj.c;
h = obj.h;

% Gaussian basis
% b = exp(-(z-c)^2/(2h))
Phi = zeros(N,K);
z = z';
for i = 1:K
    Phi(:,i) = exp(-((z-c(i)).^2)/(2*h(i)));
end
for i = 1:N
    Phi(i,:) = Phi(i,:)/sum(Phi(i,:));
end

end

