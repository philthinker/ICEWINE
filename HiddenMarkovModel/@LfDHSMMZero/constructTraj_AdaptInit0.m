function [traj,trajSigma,h,seq] = constructTraj_AdaptInit0(obj, currP, N)
%constructTraj_AdaptInit Construct a trajectory with adaptive initial state
%   currP: DP x 1, the initial position
%   N: Integer, num. of states
%   -------------------------------------------------
%   traj: DP x N, trajectory data
%   trajSigma: DP x DP x N, trajectory covariance
%   h: K x N, proability sequence
%   seq: 1 x N, state sequence
%   @LfDHSMMZero

K = obj.K;

% Initial state
s0 = obj.initialState(currP,2);
obj.StatePrior = zeros(1,K);
obj.StatePrior(s0) = 1;

[Pd, ND] = obj.durationAssign_Uniform(N,2);

h = zeros(K,N);
c = zeros(1,N); % Scaling factor to avoid numerical issues

c(1) = 1; % Initialization of scaling factor
for t=1:N
	for i=1:K
		if t<=ND
            % The 1st state duration ('s0')
			h(i,t) = obj.StatePrior(i) * Pd(i,t);
		end
		for d=1:min(t-1,ND)
			h(i,t) = h(i,t) + h(:,t-d)' * obj.Trans(:,i) * Pd(i,d);
		end
	end
	c(t+1) = 1/sum(h(:,t)+realmin); %Update of scaling factor
end
h = h ./ repmat(sum(h,1),K,1);
[~, seq] = max(h);

[traj, trajSigma] = obj.constructTraj_lscov(seq, obj.dt);

end

