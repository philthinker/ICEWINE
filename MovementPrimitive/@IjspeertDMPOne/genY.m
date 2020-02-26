function [Y] = genY(obj,x,y0,g)
%genY Generate y dy ddy.
%   x: N x 1, x, assign a scalar here for iteration
%   y0: scalar, initial position
%   g: scalar, goal position
%   Y: N x 3, [y,dy,ddy]
%   @IjspeertDMPOne

% tau dy = z
% tau dz = alpha ( beta ( y - g) - z) + f(x)

alpha = obj.alpha;
beta = obj.beta;
tau = obj.tau;
N = length(x);
dt = tau/N;

fx = obj.forcingFunc(x,y0,g);
y = zeros(N,1);
dy = zeros(N,1);
ddy = zeros(N,1);
y(1) = y0;

t = 1;
while t < N
    t = t+1;
    ddy(t) = 1/tau * 
end

Y = [y,dy,ddy];

end

