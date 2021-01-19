function [h,seq] = reconstructStSeq_StochasticSamp(obj, N)
%reconstructStSeq_StochasticSamp Reconstrunct the state sequence via
%stochastic sampling.
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
[Pd, nbD] = obj.durationAssign_Uniform(N,2);

h = zeros(K,N);

%Manual reconstruction of sequence for HSMM based on stochastic sampling
nbSt=0; currTime=0; iList=[];
while currTime<N
    nbSt = nbSt+1;
    if nbSt==1
        [~,iList(1)] = max(obj.StatePrior);
%         [~,iList(1)] = max(obj.StatePrior.*rand(model.nbStates,1));
%         iList(1) = start_state;
        h1 = ones(1,N);
    else
        h1 = [zeros(1,currTime), cumsum(Pd(iList(end-1),:)), ones(1,max(N-currTime-nbD,0))];
        currTime = currTime + round(obj.MuPd(1,iList(end-1)));
    end
    h2 = [ones(1,currTime), 1-cumsum(Pd(iList(end),:)), zeros(1,max(N-currTime-nbD,0))];
    h(iList(end),:) = h(iList(end),:) + min([h1(1:N); h2(1:N)]);
    [~,iList(end+1)] = max(obj.Trans(iList(end),:).*rand(1,K));
end
h = h ./ repmat(sum(h,1),K,1);

[~,seq] = max(h);

end

