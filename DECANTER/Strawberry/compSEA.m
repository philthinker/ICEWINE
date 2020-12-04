function [sea,areas] = compSEA(t,x,hatx)
%compSEA Compute the SEA
%   t: 1 x N, time series
%   x: 1 x N, ground truth
%   hatx: 1 x N, estimated value
%   -------------------------------------------------
%   sea: scalar, the SEA value
%   areas: 1 x N-1, SEA of each trapezoid

N = length(t);
areas = zeros(1,N-1);

for i = 2:N
    areas(i-1) = (abs(hatx(i) - x(i)) + abs(hatx(i-1) - x(i-1)))*abs(t(i) - t(i-1))/2;
end

sea = sum(areas);

end

