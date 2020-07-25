function [qOut] = hamiltonProdcut(qIn,p)
%hamiltonProdcut Hamilton product of unit quaternion p*q*p^-1
%   qIn: 4 x N, quat
%   p: 4 x N, quat
%   qOut: 4 x N, quat

qOut = quatProduct(quatProduct(p,qIn),quatconj(p')');

end

