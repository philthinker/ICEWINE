function [h,s] = reconstructStSeq_FastFW(obj,N)
%reconstructStSeq_FastFW Reconstruct the state sequence via fast forward
%computation.
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
h = zeros(K, N);

[Pd, ND] = obj.durationAssign_Uniform(N,2);

ALPHA = repmat(obj.StatesPrior, 1, ND) .* Pd;
S = obj.Trans' * ALPHA(:,1);
% alpha = sum(ALPHA,2);

for t=2:N
    % Forward computation
%     [ALPHA, S, h(:,t)] = hsmm_fwd_step_ts(model, ALPHA, S);
    %Fast computation
    ALPHA = [repmat(S(:,end), 1, ND-1) .* model.Pd(:,1:ND-1) + ALPHA(:,2:ND), ...
        S(:,end) .* Pd(:,ND)];
    % %Slow computation (but more readable)
    % for i=1:nbD-1
    %   ALPHA(:,i) = S(:,end) .* model.Pd(:,i) + ALPHA(:,i+1);	%Equation (12)
    % end
    % ALPHA(:,nbD) = S(:,end) .* model.Pd(:,nbD);
    S = [S, obj.Trans' * ALPHA(:,1)];
    alpha = sum(ALPHA, 2); %Forward variable
end

h = h ./ repmat(sum(h,1),K,1);
[~,s] = max(h);

end

