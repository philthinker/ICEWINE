function [Data] = tpDataRegulate(obj,Demos,kMeans)
%tpDataRegulate Regulate the Demos data into compact form
%   Demos: TP-Demo struct (A, b ,data, TPData)
%   kMeans: boolean, true for k-Means usage (default:false)
%   Data: (D x F) x N (kMeans == true) or D x F x N (kMeans == false)
%   A: D x D x F, orientation matrices
%   b: D x F, positions
%   @TPGMM

if nargin < 3
    kMeans = false;
end

nSample = length(Demos);
tmpN = 0;
for i = 1:nSample
    tmpN = tmpN + size(Demos{i}.data,1); 
end
if kMeans
    Data = zeros(obj.nVar*obj.nFrame,tmpN);
    tmpIndex = 1;
    for i = 1:nSample
        tmpN = size(Demos{i}.TPData,3);
        tmpData = Demos{i}.TPData;
        Data(:,tmpIndex:tmpIndex+tmpN-1) = reshape(tmpData,[size(tmpData,1)*size(tmpData,2),size(tmpData,3)]);
        tmpIndex = tmpIndex + tmpN;
    end
else
    % For EM
    Data = zeros(obj.nVar,obj.nFrame,tmpN);
    tmpIndex = 1;
    for i = 1:nSample
        tmpN = size(Demos{i}.TPData,3);
        Data(:,:,tmpIndex:tmpIndex + tmpN -1) = Demos{i}.TPData;
        tmpIndex = tmpIndex + tmpN;
    end
end


end

