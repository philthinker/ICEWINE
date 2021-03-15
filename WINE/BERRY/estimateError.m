function [Err,ErrList] = estimateError(dataEsti,dataOrig,mod)
%estimateError Compute the estimation error of two trajectories of the same
%length. 
%   dataEsti: N x D, the estimated data.
%   dataOrig: N x D, the original data.
%   mod: Integer, the estiamtion mode (default:0).
%   -------------------------------------------------
%   Err: Scalar, the estimation error.
%   ErrList: N x 1, the estimation errors of each step.

if nargin < 3
    mod = 0;
end

if mod == 1
else
    % mod == 0
    % Averaged root square error
    tmpData = dataOrig - dataEsti;
    ErrList = sqrt(diag(tmpData*tmpData'));
    Err = mean(ErrList);
end

end

