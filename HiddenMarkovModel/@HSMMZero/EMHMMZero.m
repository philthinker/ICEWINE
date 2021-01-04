function [obj,GAMMA2,LL] = EMHMMZero(obj,Demos)
%EMHMMZero Estimation of HMM parameters with an EM algorithm.
%   Demos: 1 x M cell
%   Data: D x (N x M)

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 

%Initialization of the parameters
[Data,M,nbData,nbVar] = obj.dataRegulate(Demos);    % For S. Calinon's habit
s.Data = Demos{1};
s = repmat(s,[1,M]);

for nbIter=1:obj.params_nbMaxSteps
	fprintf('.');
	
	%E-step
	for n=1:M
        
		s(n).Data = Demos{n};
        s(n).nbData = size(s(n).Data,2);
		%Emission probabilities
		for i=1:obj.K
			%s(n).B(i,:) = obj.Priors(i) * obj.GaussPDF(s(n).Data, obj.Mu(:,i), obj.Sigma(:,:,i));
			s(n).B(i,:) = obj.GaussPDF(s(n).Data, obj.Mu(:,i), obj.Sigma(:,:,i));
		end
		
		%Forward variable ALPHA (rescaled)
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
		
		%Backward variable BETA (rescaled)
		s(n).BETA(:,s(n).nbData) = ones(obj.K,1) * s(n).c(end); %Rescaling
		for t=s(n).nbData-1:-1:1
			s(n).BETA(:,t) = obj.Trans * (s(n).BETA(:,t+1) .* s(n).B(:,t+1));
			s(n).BETA(:,t) = min(s(n).BETA(:,t) * s(n).c(t), realmax); %Rescaling
		end
		
		%Intermediate variable GAMMA
		s(n).GAMMA = (s(n).ALPHA.*s(n).BETA) ./ repmat(sum(s(n).ALPHA.*s(n).BETA)+realmin, obj.K, 1); 
		
		%Intermediate variable ZETA (fast version, by considering scaling factor)
		for i=1:obj.K
			for j=1:obj.K
				s(n).ZETA(i,j,:) = obj.Trans(i,j) * (s(n).ALPHA(i,1:end-1) .* s(n).B(j,2:end) .* s(n).BETA(j,2:end)); 
			end
		end
	end
	
	%Concatenation of HMM intermediary variables
	GAMMA=[]; GAMMA_TRK=[]; GAMMA_INIT=[]; ZETA=[];
	for n=1:M
		GAMMA = [GAMMA s(n).GAMMA];
		GAMMA_INIT = [GAMMA_INIT s(n).GAMMA(:,1)];
		GAMMA_TRK = [GAMMA_TRK s(n).GAMMA(:,1:end-1)];
		ZETA = cat(3,ZETA,s(n).ZETA);
	end
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2)+realmin, 1, size(GAMMA,2));
	
	%M-step
	for i=1:obj.K
		%Update the centers
		if obj.params_updateComp(1)
			obj.Mu(:,i) = Data * GAMMA2(i,:)'; 
		end	
		%Update the covariance matrices
		if obj.params_updateComp(2)
			Data_tmp = Data - repmat(obj.Mu(:,i),1,nbData);
			obj.Sigma(:,:,i) = Data_tmp * diag(GAMMA2(i,:)) * Data_tmp'; %Eq. (54) Rabiner
			%Optional regularization term
			obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(nbVar) * obj.params_diagRegFact;
		end
	end
	
	%Update initial state probability vector
	if obj.params_updateComp(3)
		obj.StatePrior = mean(GAMMA_INIT,2);
	end
	
	%Update transition probabilities
	if obj.params_updateComp(4)
		obj.Trans = sum(ZETA,3)./ repmat(sum(GAMMA_TRK,2)+realmin, 1, obj.K); 
	end
	
	%Compute the average log-likelihood through the ALPHA scaling factors
	LL(nbIter)=0;
	for n=1:M
		LL(nbIter) = LL(nbIter) - sum(log(s(n).c));
	end
	LL(nbIter) = LL(nbIter)/M;
	%Stop the algorithm if EM converged
	if nbIter>obj.params_nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<obj.params_maxDiffLL
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end

disp(['The maximum number of ' num2str(obj.params_nbMaxSteps) ' EM iterations has been reached.']);

end

