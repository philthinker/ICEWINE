function [traj,u] = LQTBath(obj,p0,seq)
%LQTBath LQT in bath manner.
%   p0: DP x 1, the initial position
%   seq: 1 x N, the state sequence
%   -------------------------------------------------
%   traj: DP*2 x N, the trajectory generated
%   u: DP x N-1, the input signal
%   @LfDHSMMOne

N = length(seq);

% Dynamical System settings (discrete version)
A = kron([1, obj.dt; 0, 1], eye(obj.DP));
B = kron([0; obj.dt], eye(obj.DP));
C = kron([1, 0], eye(obj.DP));
%Control cost matrix
R = eye(obj.DP) * obj.r;
R = kron(eye(N-1),R);

%Build CSx and CSu matrices for batch LQR
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

MuQ = reshape(obj.Mu(:,seq), obj.DP*N, 1); 
SigmaQ = (kron(ones(N,1), eye(obj.DP)) * reshape(obj.Sigma(:,:,seq), obj.DP, obj.DP*N)) .* kron(eye(N), ones(obj.DP));

%Set matrices to compute the damped weighted least squares estimate
CSuInvSigmaQ = CSu' / SigmaQ;
Rq = CSuInvSigmaQ * CSu + R;

%Reproductions
X = [p0; zeros(obj.DP,1)];
%X = [Data(:,1); zeros(obj.DP,1)];
rq = CSuInvSigmaQ * (MuQ-CSx*X);
u = Rq \ rq; %Can also be computed with u = lscov(Rq, rq);
traj = reshape(CSx*X+CSu*u, obj.DP, N);

end

