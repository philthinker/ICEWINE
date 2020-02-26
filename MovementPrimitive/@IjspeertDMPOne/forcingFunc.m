function [fx] = forcingFunc(obj,x,y0,g)
%forcingFunc Nonlinear forcing term
%   x: N x 1, x, assign a scalar here for iteration
%   y0: scalar, initial position
%   g: scalar, goal position
%   fx: N x 1, f(x)
%   @IjspeertDMPOne

% f(x) = (sum(psi w))/(sum(psi)) x (g-y0)

N = length(x);
K = obj.nKernel;
Psi = zeros(N,K);   % N x K
w = obj.w;          % K x 1

for i = 1:K
    Psi(:,i) = obj.basisFunc(x,obj.c(i),obj.h(i));
end
fx = (Psi*w)./(Psi*ones(K,1)).*x.*(g-y0);    % N x 1

end

