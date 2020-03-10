function [Data,Ns] = dataRegulate(obj,Demos)
%dataRegulate Regulate the data in Demos into one matrix
%   Demos: 1 x M struct array:
%   |   data: D x N, demo data
%   Data: D x N*M, demo data (Individual Ns are also supported)
%   Ns: 1 x M, num. of data in each demo
%   @TrajGMMOne

M = length(Demos);
D = size(Demos(1).data,1);

Ns = zeros(1,M);
tmpIndex = 0;
for i = 1:M
    Ns(i) = size(Demos(i).data,2);
end
Data = zeros(D,sum(Ns));
for i = 1:M
    for j = 1:Ns(i)
        Data(:,tmpIndex + j) = Demos(i).data(1:D,j);
    end
    tmpIndex = tmpIndex + Ns(i);
end

end

