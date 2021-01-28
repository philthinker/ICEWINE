function [obj, GAMMA, GAMMA2] = EM_tensorHMM(obj, s)
% EM_tensorHMM  Estimation of TP-HMM parameters with an EM algorithm
%   s: 1 x M struct array
%   |   Data: DP*DD x N, the data of one demo
%   |   nbData: Integer, N
%   --------------------------------------------------
%   GAMMA: K x N, Gamma
%   GAMMA2:  K x N, Normalized Gamma
%   @TPTrajHSMM

% function [obj, GAMMA] = EM_tensorHMM(obj, s)
% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% Sylvain Calinon, 2015

%% Parameters of the EM algorithm
nbMinSteps = obj.params_nbMinSteps;      %Minimum number of iterations allowed
nbMaxSteps = obj.params_nbMaxSteps;     %Maximum number of iterations allowed
maxDiffLL = obj.params_maxDiffLL;            %Likelihood increase threshold to stop the algorithm

diagRegularizationFactor = 1E-6; %Regularization term is optional

%% Initialization of the parameters
nbSamples = length(s);

Data=[];
for n=1:nbSamples
	Data = cat(3, Data, s(n).Data);
end
nbData = size(Data,3);

%% EM
for nbIter=1:nbMaxSteps
	fprintf('.');
	
	%E-step
	for n=1:nbSamples
		
		%Emission probabilities
		s(n).B = obj.computeGamma(s(n).Data); %See 'computeGamma' function below
		
		%Forward variable ALPHA
		s(n).ALPHA(:,1) = obj.StatePrior .* s(n).B(:,1);
		%Scaling to avoid underflow issues
		s(n).c(1) = 1 / sum(s(n).ALPHA(:,1)+realmin);
		s(n).ALPHA(:,1) = s(n).ALPHA(:,1) * s(n).c(1);
		for t=2:s(n).nbData
			s(n).ALPHA(:,t) = (s(n).ALPHA(:,t-1)'*obj.Trans)' .* s(n).B(:,t); 
			%Scaling to avoid underflow issues
			s(n).c(t) = 1 / sum(s(n).ALPHA(:,t)+realmin);
			s(n).ALPHA(:,t) = s(n).ALPHA(:,t) * s(n).c(t);
		end
		
		%Backward variable BETA
		s(n).BETA(:,s(n).nbData) = ones(obj.K,1) * s(n).c(end); %Rescaling
		for t=s(n).nbData-1:-1:1
			s(n).BETA(:,t) = obj.Trans * (s(n).BETA(:,t+1) .* s(n).B(:,t+1));
			s(n).BETA(:,t) = min(s(n).BETA(:,t) * s(n).c(t), realmax); %Rescaling
		end
		
		%Intermediate variable GAMMA
		s(n).GAMMA = (s(n).ALPHA.*s(n).BETA) ./ repmat(sum(s(n).ALPHA.*s(n).BETA)+realmin, obj.K, 1); 
		
		%Intermediate variable XI (fast version, by considering scaling factor)
		for i=1:obj.K
			for j=1:obj.K
				s(n).XI(i,j,:) = obj.Trans(i,j) * (s(n).ALPHA(i,1:end-1) .* s(n).B(j,2:end) .* s(n).BETA(j,2:end)); 
			end
		end
	end
	
	%Concatenation of HMM intermediary variables
	GAMMA=[]; GAMMA_TRK=[]; GAMMA_INIT=[]; XI=[];
	for n=1:nbSamples
		GAMMA = [GAMMA s(n).GAMMA];
		GAMMA_INIT = [GAMMA_INIT s(n).GAMMA(:,1)];
		GAMMA_TRK = [GAMMA_TRK s(n).GAMMA(:,1:end-1)];
		XI = cat(3,XI,s(n).XI);
	end
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2)+realmin, 1, size(GAMMA,2));
	
	%M-step
	for i=1:obj.K	
		for m=1:obj.F
			%Matricization/flattening of tensor
			DataMat(:,:) = Data(:,m,:);
			
			%Update Mu
			obj.Mus(:,m,i) = DataMat * GAMMA2(i,:)';
			
			%Update Sigma (regularization term is optional)
			DataTmp = DataMat - repmat(obj.Mus(:,m,i),1,nbData);
			obj.Sigmas(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(obj.D) * diagRegularizationFactor;
		end
    end
	
    % For HMM transition param.
    
	%Update initial state probability vector
	obj.StatePrior = mean(GAMMA_INIT,2); 
	
	%Update transition probabilities
	obj.Trans = sum(XI,3)./ repmat(sum(GAMMA_TRK,2)+realmin, 1, obj.K); 
	
	%Compute the average log-likelihood through the ALPHA scaling factors
	LL(nbIter)=0;
	for n=1:nbSamples
		LL(nbIter) = LL(nbIter) - sum(log(s(n).c));
	end
	LL(nbIter) = LL(nbIter)/nbSamples;
	%Stop the algorithm if EM converged
	if nbIter>nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<maxDiffLL
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end
disp(['The maximum number of ' num2str(nbMaxSteps) ' EM iterations has been reached.']);
end
