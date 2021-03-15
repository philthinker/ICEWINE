function [Err,ErrList] = estimateQuatError(dataEsti,dataOrig)
%estimateQuatError Compute the estimation error of two unit quaternion
%trajectories of the same length.
%   dataEsti: N x 4, the estimated data [w x y z].
%   dataOrig: N x 4, the original data [w x y z].
%   -------------------------------------------------
%   Err: Scalar, the estimation error.
%   ErrList: N x 1, the estimation errors of each step.

%% Data regulate
N = size(dataEsti,1);
dataEsti = quatNormalize(dataEsti(:,1:4)');        % 4 x N
dataOrig = quatNormalize(dataOrig(:,1:4)');     % 4 x N
tmpLogicIDs = dataEsti(1,:) < 0;
dataEsti(:,tmpLogicIDs) = -dataEsti(:,tmpLogicIDs);
tmpLogicIDs = dataOrig(1,:) < 0;
dataOrig(:,tmpLogicIDs) = -dataOrig(:,tmpLogicIDs);

%% Error computation
% Refer to "Morais, João Pedro, Svetlin Georgiev, and Wolfgang Sprößig.
% Real quaternionic calculus handbook. Springer Basel, 2014."
dataEstiConj = dataEsti;
dataEstiConj(2:4,:) = -dataEsti(2:4,:);
ErrList = zeros(N,1);
qErr = quatProduct(dataOrig,dataEstiConj); % 4 x N
for i = 1:N
    if all(qErr(:,i) == [-1, 0, 0, 0]')
        ErrList(i) = 2*pi;
    else
        ErrList(i) = 2*norm(quatlog(qErr(:,i)'));
    end
end
Err = mean(ErrList);

end

