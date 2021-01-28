function [DataOut, SigmaOut, obj] = constructTraj(obj,seq,frames)
%constructTraj Construct the trajectory based on traj-HSMM and Gaussian
%products across each frames.
%   seq: 1 x N, state sequence
%   frames: 1 x F struct array:
%   |   A: D x D, A matrices
%   |   b: D x 1, b vectors
%   -------------------------------------------------
%   DataOut: DP x N, expected position data
%   SigmaOut: DP x DP x N, expected covariance data
%   obj
%   @TPTrajHSMMZero

% Products of linearly transformed Gaussians
[Mu, Sigma, obj] = obj.prodLinearTransformedGauss(frames);

% State sequence
N = length(seq);
MuQ = reshape(Mu(:,seq), obj.DP*obj.DD*N, 1);
SigmaQ = zeros(obj.DP*obj.DD*N);
for t=1:N
    id1 = (t-1)*obj.DP*obj.DD+1:t*obj.DP*obj.DD;
    SigmaQ(id1,id1) = Sigma(:,:,seq(t));
end

% Trajectory HMM
PHI1 = obj.constructPhi1(N,obj.dt);
PHIinvSigmaQ = PHI1'/SigmaQ;
Rq = PHIinvSigmaQ * PHI1;
rq = PHIinvSigmaQ * MuQ;
DataOut = reshape(Rq\rq, obj.DP, N); %Reshape data for plotting

% Covariance Matrix computation of ordinary least squares estimate
tmpMuQInvSigmaQ = MuQ'/SigmaQ;
tmprqInvRq = rq'/Rq;
mse =  (tmpMuQInvSigmaQ*MuQ - tmprqInvRq*rq) ./ ((obj.D-obj.DP)*N);
S = Rq\mse;
SigmaOut = repmat(eye(obj.DP),[1,1,N]);
for t=1:N
    id = (t-1)*obj.DP+1:t*obj.DP;
    SigmaOut(:,:,t) = S(id,id)*N;
end

end

