function [Data,legal] = dataFlattening(obj,TPDemos,mode)
%dataFlattening Flatten the demo data for K-Means or EM algorithm
%   TPDemos: 1 x M TPDemo struct array, the demos
%   mode: integer, 0 for EM (default) and 1 for K-Means
%   Data: (D*F) x N (mode == 1) or D x F x N (mode == 0) (default), demo data
%   legal: boolean, true for the features of demo data is identical to
%   those of the TPGMM.
%   @TPGMMOne

if nargin < 3
    mode = 0;
end

D = size(TPDemos(1).TPData,1);
F = size(TPDemos(1).TPData,2);
legal = D == obj.nVar && F == obj.nFrame;

M = length(TPDemos);
tmpN = 0;
for i = 1:M
    tmpN = tmpN + size(TPDemos(i).TPData,3); 
end
if mode == 1
    % For K-Means
    Data = zeros(D*F,tmpN);
    tmpIndex = 1;
    for i = 1:M
        tmpN = size(TPDemos(i).TPData,3);
        tmpData = TPDemos(i).TPData;
        Data(:,tmpIndex:tmpIndex+tmpN-1) = reshape(tmpData,[size(tmpData,1)*size(tmpData,2),size(tmpData,3)]);
        tmpIndex = tmpIndex + tmpN;
    end
else
    % For EM
    Data = zeros(D,F,tmpN);
    tmpIndex = 1;
    for i = 1:M
        tmpN = size(TPDemos(i).TPData,3);
        Data(:,:,tmpIndex:tmpIndex + tmpN -1) = TPDemos(i).TPData;
        tmpIndex = tmpIndex + tmpN;
    end
end

end