function [Data,N] = demoFlatten(obj,Demos)
%demoFlatten Flatten the demos in cells into a large array
%   Demos: 1 x M cell, the demos
%   Data: D x N, the large data array
%   N: integer, num.of data in total
%   @GMMSake

M = length(Demos);
D = size(Demos{1},1);
N = 0;

tempN = zeros(1,M);

for i = 1:M
    tempN(i) = size(Demos{i},2);
    N = N + tempN(i);
end

Data = zeros(D,N);
tempIndex = 1;

for i = 1:M
    Data(:,tempIndex:tempIndex+tempN(i)-1) = Demos{i};
    tempIndex = tempIndex + tempN(i);
end

end

