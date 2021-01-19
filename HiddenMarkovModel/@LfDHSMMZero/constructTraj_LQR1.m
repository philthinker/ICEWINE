function [traj,h,seq] = constructTraj_LQR1(obj,currP,N)
%constructTraj_LQR1 Construct a trajectory via LQR.
%   currP: DP x 1, current position
%   N: Integer, num. of data points
%   -------------------------------------------------
%   traj: DP x N, trajectory ouput
%   h: K' x N, K' <= K, the probabilities
%   seq: 1 x N, the state sequence
%   @LfDHSMMZero

%% Initial state and seq. assignment

% hsmm
initSt = obj.initialState(currP);
obj.StatePrior = zeros(1, obj.K); obj.StatePrior(initSt) = 1;
[h,seq] = obj.reconstructStSeq_StandardFW(N);

%% LQT
%Dynamical System settings (discrete version)
A = kron([1, obj.dt; 0, 1], eye(obj.DP));
B = kron([0; obj.dt], eye(obj.DP));
C = kron([1, 0], eye(obj.DP));
%Control cost matrix
R = eye(obj.DP) * obj.r;
R = kron(eye(N-1),R);

%Build CSx and CSu matrices for batch LQR, see Eq. (35)
CSu = zeros(obj.DP*N, obj.DP*(N-1));
CSx = kron(ones(N,1), [eye(obj.DP) zeros(obj.DP)]);
M = B;
for n=2:N
	id1 = (n-1)*obj.DP+1:n*obj.DP;
	CSx(id1,:) = CSx(id1,:) * A;
	id1 = (n-1)*obj.DP+1:n*obj.DP; 
	id2 = 1:(n-1)*obj.DP;
	CSu(id1,id2) = C * M;
	M = [A*M(:,1:obj.DP), M];
end

% %Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (27)
% [~,qList] = max(h,[],1); %works also for nbStates=1

% n=2;
% qList = h((n-1)*N+1:n*N); %estimation from data (instead of HSMM parameters)
qList = seq;

MuQ = reshape(obj.Mu(:,qList), obj.DP*N, 1); 
SigmaQ = (kron(ones(N,1), eye(obj.DP)) * reshape(obj.Sigma(:,:,qList), obj.DP, obj.DP*N)) .* kron(eye(N), ones(obj.DP));

%Set matrices to compute the damped weighted least squares estimate
CSuInvSigmaQ = CSu' / SigmaQ;
Rq = CSuInvSigmaQ * CSu + R;

%Reproductions

X = [currP; zeros(obj.DP,1)];
%X = [Data(:,1); zeros(obj.DP,1)];
rq = CSuInvSigmaQ * (MuQ-CSx*X);
u = Rq \ rq; %Can also be computed with u = lscov(Rq, rq);
traj = reshape(CSx*X+CSu*u, obj.DP, N);

end

