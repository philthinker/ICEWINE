function [h,seq] = reconstructStSeq_StandardFW(obj, N)
%reconstructStSeq_StandardFW Reconstrunct the state sequence via standard
%forward computation
%   N: Integer, num. of state data required
%   -----------------------------------------
%   h: K x N, state probabilities
%   seq: 1 x N, state sequence
%   @HSMMZero

% 
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

K = obj.K;
[Pd, ND] = obj.durationAssign_Uniform(N,2);

h = zeros(K,N);
c = zeros(1,N); % Scaling factor to avoid numerical issues

c(1) = 1; % Initialization of scaling factor
for t=1:N
	for i=1:K
		if t<=ND
            % The 1st state duration
            % Observation probability for generative purpose
			oTmp = 1; 
            % Observation probability for standard HSMM
% 			oTmp = prod(c(1:t) .* gaussPDF(s(1).Data(:,1:t), obj.Mu(:,i), obj.Sigma(:,:,i))'); 
			h(i,t) = obj.StatePrior(i) * Pd(i,t) * oTmp;
		end
		for d=1:min(t-1,ND)
            % Observation probability for generative purpose
			oTmp = 1; 
            % Observation probability for standard HSMM	
% 			oTmp = prod(c(t-d+1:t) .* gaussPDF(s(1).Data(:,t-d+1:t), obj.Mu(:,i), obj.Sigma(:,:,i))'); 
			h(i,t) = h(i,t) + h(:,t-d)' * obj.Trans(:,i) * Pd(i,d) * oTmp;
		end
	end
	c(t+1) = 1/sum(h(:,t)+realmin); %Update of scaling factor
end
h = h ./ repmat(sum(h,1),K,1);
[~, seq] = max(h);

end

