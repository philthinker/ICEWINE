function [traj] = SampleHSMMLQR_SC(obj, p0, N)
%SampleHSMMLQR_SC Sampling based iterative LQR controller for HSMM
%   p0: DP x 1, the current/initial position
%   N: Integer, the num. of states required
%   --------------------------------------------------
%   traj: DP*2 x N, the trajectory generated (position and velocity)
%   @LfDHSMMOne

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 

% if nargin<4
%     currP = obj.Mu(:,initSt);
% end

K = obj.K;
DP = obj.DP;

nbData = N; %
nbD = round(2 * nbData/K); %Number of maximum duration step to consider in the HSMM (2 is a safety factor)
%Precomputation of duration probabilities
Pd = zeros(1,nbD);
for i=1:K
    Pd(i,:) = obj.GaussPDF((1:nbD), obj.MuPd(:,i), obj.SigmaPd(:,:,i));
    %The rescaling formula below can be used to guarantee that the cumulated sum is one (to avoid the numerical issues)
    Pd(i,:) = Pd(i,:) / sum(Pd(i,:));
end

% Determine the initial state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
closeness = zeros(K,1); % find "closest" state
for i=1:K
    closeness(i,:)=obj.GaussPDF(p0, obj.Mu(1:DP,i), obj.Sigma(1:DP, 1:DP, i));
end
[~,initSt] = max(closeness);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Manual reconstruction of sequence for HSMM based on stochastic sampling
nbSt=0; currTime=0; iList=[];
h = zeros(K,nbData);
while currTime<nbData
    nbSt = nbSt+1;
    if nbSt==1
        %[~,iList(1)] = max(model.StatesPriors.*rand(model.nbStates,1));
        iList(1) = initSt;
        h1 = ones(1,nbData);
    else
        h1 = [zeros(1,currTime), cumsum(Pd(iList(end-1),:)), ones(1,max(nbData-currTime-nbD,0))];
        currTime = currTime + round(obj.MuPd(1,iList(end-1)));
    end
    h2 = [ones(1,currTime), 1-cumsum(Pd(iList(end),:)), zeros(1,max(nbData-currTime-nbD,0))];
    h(iList(end),:) = h(iList(end),:) + min([h1(1:nbData); h2(1:nbData)]);
    [~,iList(end+1)] = max(obj.Trans(iList(end),:).*rand(1,K));
end
h = h ./ repmat(sum(h,1),K,1);

[~,qList] = max(h);

% Iterative LQR reproduction (finite horizon)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Dynamical System settings (discrete version)
A = kron([1, obj.dt; 0, 1], eye(obj.DP));
B = kron([0; obj.dt], eye(obj.DP));
%C = kron([1, 0], eye(model.nbVarPos));
%Control cost matrix
R = eye(obj.DP) * obj.r;

P = zeros(obj.DP*2,obj.DP*2,nbData);
P(1:obj.DP,1:obj.DP,end) = inv(obj.Sigma(:,:,qList(nbData)));
d = zeros(obj.DP*2, nbData);
Q = zeros(obj.DP*2);
for t=nbData-1:-1:1
    Q(1:obj.DP,1:obj.DP) = inv(obj.Sigma(:,:,qList(t)));
    P(:,:,t) = Q - A' * (P(:,:,t+1) * B / (B' * P(:,:,t+1) * B + R) * B' * P(:,:,t+1) - P(:,:,t+1)) * A;
    d(:,t) = (A' - A'*P(:,:,t+1) * B / (R + B' * P(:,:,t+1) * B) * B' ) * (P(:,:,t+1) * ...
        ( A * [obj.Mu(:,qList(t)); zeros(obj.DP,1)] - [obj.Mu(:,qList(t+1)); zeros(obj.DP,1)] ) + d(:,t+1));
end
%Reproduction with feedback (FB) and feedforward (FF) terms
X = [p0; zeros(obj.DP,1)]; %[model.Mu(:,qList(1)); zeros(model.nbVarPos,1)];
r.X0 = X;
for t=1:nbData
    r.Data(:,t) = X; %Log data
    K = (B' * P(:,:,t) * B + R) \ B' * P(:,:,t) * A; %FB term
    M = -(B' * P(:,:,t) * B + R) \ B' * (P(:,:,t) * ...
        (A * [obj.Mu(:,qList(t)); zeros(obj.DP,1)] - [obj.Mu(:,qList(t)); zeros(obj.DP,1)]) + d(:,t)); %FF term
    u = K * ([obj.Mu(:,qList(t)); zeros(obj.DP,1)] - X) + M; %Acceleration command with FB and FF terms
    X = A * X + B * u; %Update of state vector
end

traj = r.Data;
% plot(r(n).Data(1,:),r(n).Data(2,:),'-b','LineWidth',2);

end