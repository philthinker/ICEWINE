function [DataOut, SigmaOut, obj] = constructTraj_lscov(obj,seq,frames)
%constructTraj_lscov Construct the trajectory via MATLAB lscov func.
%   seq: 1 x N, state sequence
%   frames: 1 x F struct array:
%   |   A: D x D, A matrices
%   |   b: D x 1, b vectors
%   -----------------------------------------
%   DataOutput: DP x N, expected position data
%   SigmaOut: DP x DP x N, expected position covariance
%   obj
%   @TPTrajHSMMZero

% Products of linearly transformed Gaussians
[Mu, Sigma, obj] = obj.prodLinearTransformedGauss(frames);

N = length(seq);
dt = obj.dt;

MuQ = reshape(Mu(:,seq), obj.D * N, 1);
SigmaQ = ( kron(ones(N,1), eye(obj.D)) * reshape(Sigma(:,:,seq), obj.D, obj.D*N))...
    .* kron(eye(N), ones(obj.D));
% % Or
% SigmaQ = zeros(obj.N*D);
% for t=1:N
% 	id = (t-1)*obj.D+1:t*obj.D;
% 	%MuQ(id) = obj.Mu(:,Seq(t));
% 	SigmaQ(id,id) = obj.Sigma(:,:,Seq(t));
% end
PHI1 = obj.constructPhi1(N,dt);

% Least squares via lscov Matlab function
[xhat,~,~,S] = lscov(PHI1, MuQ, SigmaQ, 'chol'); % Retrieval of data with weighted least squares solution
DataOut = reshape(xhat, obj.DP, N); % Reshape data for plotting

% % Least squares computation method 2 (most readable but not optimized)
% PHIinvSigmaQ = PHI1' / SigmaQ;
% Rq = PHIinvSigmaQ * PHI1;
% rq = PHIinvSigmaQ * MuQ;
% xhat = Rq \ rq; % Can also be computed with c = lscov(Rq, rq)
% DataOut = reshape(xhat, obj.DP, N); % Reshape data for plotting
% % Covariance Matrix computation of ordinary least squares estimate
% mse =  (MuQ'*inv(SigmaQ)*MuQ - rq'*inv(Rq)*rq) ./ ((obj.D-obj.DP)*N);
% S = inv(Rq) * mse;

SigmaOut = repmat(eye(obj.DP),[1,1,N]);
for t=1:N
    id = (t-1)*obj.DP+1:t*obj.DP;
    SigmaOut(:,:,t) = S(id,id) * N;
end

end