function [indices] = criticalPointsAssign(N,Np)
%criticalPointsAssign Critical points assignment
%   N: integer, the total number of data
%   Np: integer, the number of points required
%   -------------------------------------------------
%   indices: 1 x Np, the indices assigned

indices = linspace(1,N,Np);
indices = round(indices);

end

