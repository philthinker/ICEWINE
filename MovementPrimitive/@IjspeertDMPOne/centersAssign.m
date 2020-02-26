function [centers,variances] = centersAssign(obj,h)
%centersAssign Assign the centers and reset the variances
%   h: scalar, variance.
%   centers: K x 1, centers, c.
%   variances: K x 1, variances, h.
%   @IjspeertDMPOne
%
%   psi = exp(-(x-c)^2/2h)

N = 1000;
x = obj.genX(N);
L = floor(N/obj.nKernel);
tmpIndex = false(N,1);
tmpIndex(1) = true;
for i = 2:N-L
    if mod(i,L) == 0
        tmpIndex(i) = true;
    end
end

centers = x(tmpIndex);
variances = (centers./(obj.nKernel * h)).^2;


end

