function [Data] = dataFlattening(obj,TPDemos,mode)
%dataFlattening Flatten the demo data for K-Means or EM algorithm
%   TPDemos: 1 x M TPDemo struct array, the demos
%   mode: integer, 0 for EM (default) and 1 for K-Means
%   Data: D x F x N (mode == 1) or (D * F) x N (mode == 0), demo data
%   @TPGMMOne

if nargin < 3
    mode = 0;
end

M = length(TPDemos);
tmpN = 0;
for i = 1:M
    tmpN = tmpN + size(TPDemos(i).TPData,3); 
end
if mode == 1
    % For K-Means
    Data = zeros(obj.nVar*obj.nFrame,tmpN);
    tmpIndex = 1;
    for i = 1:M
        tmpN = size(TPDemos(i).TPData,3);
        tmpData = TPDemos(i).TPData;
        Data(:,tmpIndex:tmpIndex+tmpN-1) = reshape(tmpData,[size(tmpData,1)*size(tmpData,2),size(tmpData,3)]);
        tmpIndex = tmpIndex + tmpN;
    end
else
    % For EM
    Data = zeros(obj.nVar,obj.nFrame,tmpN);
    tmpIndex = 1;
    for i = 1:M
        tmpN = size(TPDemos(i).TPData,3);
        Data(:,:,tmpIndex:tmpIndex + tmpN -1) = TPDemos(i).TPData;
        tmpIndex = tmpIndex + tmpN;
    end
end

end

