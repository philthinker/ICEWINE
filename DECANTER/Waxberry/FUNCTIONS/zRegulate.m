function [zOut,I] = zRegulate(zIn)
%zRegulate Regulate the z data to plot
%   zIn: 1 x N
%   zOut: 1 x N
%   I: 1 x N, index

zIn(abs(zIn) < 1e-4) = 0.0;
[zOut,I] = sort(zIn);

end

