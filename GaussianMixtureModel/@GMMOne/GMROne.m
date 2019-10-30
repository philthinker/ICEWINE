function [expData,expSigma] = GMROne(obj,query,in,out)
%GMROne Gaussian Mixture Regression
%   query: Din x N, query sequence
%   in: Din x 1, indices of query variable
%   out: Dout x 1, indices of regression variable
%   expData: Dout x N, regression data
%   expSigma: Dout x Dout x N, regression covariances
%   @GMMOne

%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

nbData = size(query,2);
nbVarOut = length(out);
diagRegularizationFactor = 1E-8; %Optional regularization term

MuTmp = zeros(nbVarOut,obj.nKernel);
expData = zeros(nbVarOut,nbData);
expSigma = zeros(nbVarOut,nbVarOut,nbData);
for t=1:nbData
	%Compute activation weight
	for i=1:obj.nKernel
		H(i,t) = obj.Prior(i) * obj.GaussPDF(query(:,t), obj.Mu(in,i), obj.Sigma(in,in,i));
	end
	H(:,t) = H(:,t) / sum(H(:,t)+realmin);
	%Compute conditional means
	for i=1:obj.nKernel
		MuTmp(:,i) = obj.Mu(out,i) + obj.Sigma(out,in,i)/obj.Sigma(in,in,i) * (query(:,t)-obj.Mu(in,i));
		expData(:,t) = expData(:,t) + H(i,t) * MuTmp(:,i);
	end
	%Compute conditional covariances
	for i=1:obj.nKernel
		SigmaTmp = obj.Sigma(out,out,i) - obj.Sigma(out,in,i)/obj.Sigma(in,in,i) * obj.Sigma(in,out,i);
		expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
	end
	expSigma(:,:,t) = expSigma(:,:,t) - expData(:,t)*expData(:,t)' + eye(nbVarOut) * diagRegularizationFactor; 
end

end

