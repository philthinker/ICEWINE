function [L, GAMMA] = computeGamma(obj,Data)
%computeGamma 此处显示有关此函数的摘要
%   Data: D x (N * M), All the demonstration data
%   @GMMOne

L = zeros(obj.nKernel,size(Data,2));

for i=1:obj.nKernel
	L(i,:) = obj.Prior(i) * obj.GaussPDF(Data, obj.Mu(:,i), obj.Sigma(:,:,i));
end
GAMMA = L ./ repmat(sum(L,1)+realmin, obj.nKernel, 1);

end

