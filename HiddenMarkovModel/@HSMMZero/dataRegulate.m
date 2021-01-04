function [Data,M,N,D] = dataRegulate(obj,Demos)
%dataRegulate Regulate the data in Demos into one matrix
%   Demos: 1 x M cell, the demos
%   @HSMMZero

M = length(Demos);
Ns = ones(1,M);
D = size(Demos{1},1);
for i = 1:M
    Ns(i) = size(Demos{i},2);
end
N = sum(Ns);
Data = zeros(D,N);

tmpIndex = 1;
for i = 1:M
    Data(:,tmpIndex:tmpIndex+Ns(i)-1) = Demos{i};
    tmpIndex = tmpIndex + Ns(i);
end

end

