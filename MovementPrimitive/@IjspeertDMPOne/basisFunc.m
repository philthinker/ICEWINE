function [psi] = basisFunc(obj,x,c,h)
%basisFunc Basis func.
%   x: N x 1, x, you can assign a scalar here for iterative usage
%   c: scalar, the center
%   psi: N x 1, psi
%   @IjspeertDMPOne

% psi = exp(-(x - c)^2/(2h))

psi = exp(-(x-c).^2/(2*h));

end

