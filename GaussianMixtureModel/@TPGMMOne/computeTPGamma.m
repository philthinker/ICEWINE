function [Lik, GAMMA, GAMMA0] = computeTPGamma(obj,Data)
%computeTPGamma Compute the Gamma in E step of EM algorithm
%   Data: D x F x N, demo data
%   @TPGMMOne

nData = size(Data, 3);
Lik = ones(obj.nKernel, nData);
GAMMA0 = zeros(obj.nKernel, obj.nFrame, nData);
for i=1:obj.nKernel
	for m=1:obj.nFrame
		DataMat(:,:) = Data(:,m,:); %Matricization/flattening of tensor
        GAMMA0(i,m,:) = obj.GaussPDF(DataMat, obj.Mus(:,m,i), obj.Sigmas(:,:,m,i));
		Lik(i,:) = Lik(i,:) .* squeeze(GAMMA0(i,m,:))';
	end
	Lik(i,:) = Lik(i,:) * obj.Prior(i);
end
GAMMA = Lik ./ repmat(sum(Lik,1)+realmin, size(Lik,1), 1);

end

