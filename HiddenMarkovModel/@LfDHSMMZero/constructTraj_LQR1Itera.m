function [traj,h,seq] = constructTraj_LQR1Itera(obj,currP,N)
%constructTraj_LQR1Itera Construct a trajectory via LQR iteratively.
%   currP: DP x 1, current position
%   N: Integer, num. of data points
%   -------------------------------------------------
%   traj: DP x N, trajectory ouput
%   h: K' x N, K' <= K, the probabilities
%   seq: 1 x N, the state sequence
%   @LfDHSMMZero

%% Initial state assignment

K = obj.K;
DP = obj.DP;
DD = obj.DD;
initSt = obj.initialState(currP);
obj.StatePrior = zeros(obj.K,1); obj.StatePrior(initSt) = 1;

%% State sequence

[h, seq] = obj.reconstructStSeq_StochasticSamp(N);
qList = seq;

%% LQR

%Dynamical System settings (discrete version)
A = kron([1, obj.dt; 0, 1], eye(DP));
B = kron([0; obj.dt], eye(DP));
%C = kron([1, 0], eye(DP));
%Control cost matrix
R = eye(DP) * obj.r;

P = zeros(DP*2,DP*2,N);
P(1:DP,1:DP,end) = inv(obj.Sigma(:,:,qList(N)));
d = zeros(DP*2, N);
Q = zeros(DP*2);
for t=N-1:-1:1
    Q(1:DP,1:DP) = inv(obj.Sigma(:,:,qList(t)));
    P(:,:,t) = Q - A' * (P(:,:,t+1) * B / (B' * P(:,:,t+1) * B + R) * B' * P(:,:,t+1) - P(:,:,t+1)) * A;
    d(:,t) = (A' - A'*P(:,:,t+1) * B / (R + B' * P(:,:,t+1) * B) * B' ) * (P(:,:,t+1) * ...
        ( A * [obj.Mu(:,qList(t)); zeros(DP,1)] - [obj.Mu(:,qList(t+1)); zeros(DP,1)] ) + d(:,t+1));
end
%Reproduction with feedback (FB) and feedforward (FF) terms
X = [currP; zeros(DP,1)]; %[obj.Mu(:,qList(1)); zeros(DP,1)];
r.X0 = X;
for t=1:N
    r.Data(:,t) = X; %Log data
    K = (B' * P(:,:,t) * B + R) \ B' * P(:,:,t) * A; %FB term
    M = -(B' * P(:,:,t) * B + R) \ B' * (P(:,:,t) * ...
        (A * [obj.Mu(:,qList(t)); zeros(DP,1)] - [obj.Mu(:,qList(t)); zeros(DP,1)]) + d(:,t)); %FF term
    u = K * ([obj.Mu(:,qList(t)); zeros(DP,1)] - X) + M; %Acceleration command with FB and FF terms
    X = A * X + B * u; %Update of state vector
end

traj = r.Data;

end

