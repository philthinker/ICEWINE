function [Lik, GAMMA, GAMMA0] = computeGamma(obj, Data)
%computeGamma Compute Gamma
%   Data: D x N, data
%   -------------------------------------------------
%   Lik: K x N, likelihood
%   GAMMA: 1 x N, Gamma
%   GAMMA0: 1 x N, Gamma0
%   @TPTrajHSMMZero

nbData = size(Data, 3);
Lik = ones(obj.K, nbData);
GAMMA0 = zeros(obj.K, obj.F, nbData);
for i=1:obj.K
    for m=1:obj.F
        DataMat(:,:) = Data(:,m,:); %Matricization/flattening of tensor
        GAMMA0(i,m,:) = obj.GaussPDF(DataMat, obj.Mus(:,m,i), obj.Sigmas(:,:,m,i));
        Lik(i,:) = Lik(i,:) .* squeeze(GAMMA0(i,m,:))';
    end
    Lik(i,:) = Lik(i,:) * obj.Prior(i);
end
GAMMA = Lik ./ repmat(sum(Lik,1)+realmin, size(Lik,1), 1);
end

