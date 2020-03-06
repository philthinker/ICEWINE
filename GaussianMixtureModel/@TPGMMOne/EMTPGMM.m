function [obj, GAMMA0, GAMMA2] = EMTPGMM(obj,Data)
%EMTPGMM Training of a task-parameterized Gaussian mixture model (GMM) with an expectation-maximization (EM) algorithm.
% The approach allows the modulation of the centers and covariance matrices of the Gaussians with respect to
% external parameters represented in the form of candidate coordinate systems.
%   Data: D x F x N, demo data
%   GAMMA0: GAMMA in E step
%   GAMMA2: GAMMA for TP-GMM
%   @TPGMMOne

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

%Parameters of the EM algorithm
nbMinSteps = obj.params_nbMinSteps; %Minimum number of iterations allowed
nbMaxSteps = obj.params_nbMaxSteps; %Maximum number of iterations allowed
obj.params_maxDiffLL = 1E-5;
maxDiffLL = obj.params_maxDiffLL; %Likelihood increase threshold to stop the algorithm
nData = size(Data,3);

diagRegularizationFactor = 1E-8;  %Optional regularization term

for nbIter=1:nbMaxSteps
	fprintf('.');
    
	%E-step
    [L, GAMMA, GAMMA0] = obj.computeTPGamma(Data);
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nData);
	obj.Pix = GAMMA2;
	
	%M-step
	for i=1:obj.nKernel
		
		%Update Priors
		obj.Prior(i) = sum(sum(GAMMA(i,:))) / nData;
		
		for m=1:obj.nFrame
			%Matricization/flattening of tensor
			DataMat(:,:) = Data(:,m,:);

			%Update Mu
			obj.Mus(:,m,i) = DataMat * GAMMA2(i,:)';

			%Update Sigma (regularization term is optional)
			DataTmp = DataMat - repmat(obj.Mus(:,m,i),1,nData);
			obj.Sigmas(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(size(DataTmp,1)) * diagRegularizationFactor;
		end
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(L,1))) / size(L,2);
	%Stop the algorithm if EM converged (small change of LL)
	if nbIter>nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<maxDiffLL || nbIter==nbMaxSteps-1
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end
disp(['The maximum number of ' num2str(nbMaxSteps) ' EM iterations has been reached.']);
end

