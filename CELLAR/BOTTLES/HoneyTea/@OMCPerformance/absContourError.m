function [e] = absContourError(obj,x,XR)
%absContourError Absolute contour error
%   x: 1 x 3, point position
%   XR: N x 3, reference trajectory
%   @OMCPerformance

E = abs(x - XR)';
e = min(sqrt(diag(E'*E)));

end

