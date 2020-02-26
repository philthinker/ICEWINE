function [x] = genX(obj,N)
%genX Generate the x sequence based 
%   N: integer, num. of data
%   x: N x 1, x sequence from 1 to 0 (nearly)
%   @IjspeertDMPOne  

N = round(N);
if obj.mode == 1
    % Linear sequence
    x = linspace(0,1,N)';
else
    % tau dx = -alphax x
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

end

