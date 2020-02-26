function [x] = genX(obj,N)
%genX Generate the x sequence based 
%   N: integer, num. of data
%   x: N x 1, x sequence from 1 to 0 (nearly)
%   @IjspeertDMPOne
%
%   tau dx = -alphax x

N = round(N);
tau = obj.tau;
dt = tau/N;
alphax = obj.alphax;
x = ones(N,1);

t = 1;
while t < N
    t = t+1;
    dx = -(1/tau) * alphax * x(t-1);
    x(t) = x(t-1) + dx * dt; 
end

end

