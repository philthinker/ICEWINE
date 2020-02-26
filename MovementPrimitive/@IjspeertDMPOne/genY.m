function [Y,fx] = genY(obj,x,y0,g)
%genY Generate y dy ddy.
%   x: N x 1, x
%   y0: scalar, initial position
%   g: scalar, goal position
%   Y: N x 3, [y,dy,ddy]
%   fx: N x 1, forcing term
%   @IjspeertDMPOne

% tau dy = z
% tau dz = alpha ( beta (g - y) - z) + f(x)

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

% Note: z = tau dy; dz = tau ddy
t = 1;
while t < N
    t = t+1;
    ddy(t-1) = 1/(tau^2) * (alpha * (beta * (g - y(t-1)) - tau * dy(t-1)) + fx(t-1));
    dy(t) = dy(t-1) + dt * ddy(t-1);
    y(t) = y(t-1) + dt * dy(t-1);
end

Y = [y,dy,ddy];

end

