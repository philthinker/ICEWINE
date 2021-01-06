classdef TrajHSMMZero < HSMMZero
    %TrajHSMMZero Trajectory Hidden Markov Model
    %   Initialization with dim. of position, order of derivation and num. of states.
    %   It is designed specifically for robot PbD applications.
    %
    %   Haopeng Hu
    %   2021.01.06
    %   All rights reserved.
    %
    %   Notations:
    %   |   D:  Dim. of data
    %   |   DP: Dim. of position
    %   |   DD: Order of derivation
    %   |   N:  Num. of data
    %   |   K:  Num. of kernels
    %   |   M: Num. of demos
    %
    %   Recommendation:
    %   1   obj = TrajHSMMZero(DP,DD,K);
    %   2   obj = obj.initHMM...
    %   3   obj = obj.initTrans...
    %   4   obj = obj.leanHMM...
    %   5   [~,s] = obj.reconstructStSeq_...
    %   6   [Data] = obj.constructTraj_...
    
    properties
        DP;         % Dim. of position 
        DD;         % Order of derivation
    end
    
    methods
        function obj = TrajHSMMZero(DP,DD,K)
            %TrajHSMMZero Initialization with dim. of position, order of derivation and num. of states.
            %   DP: Integer, DP
            %   DD: Integer, DD
            %   K: Integer, K
            DP = round(max([1, DP(1,1)]));
            DD = round(max([1, DD(1,1)]));
            obj = obj@HSMMZero(DP*DD,K);
            obj.DP = DP;
            obj.DD = DD;
        end
        
        function [DataOut, SigmaOut] = constructTraj_lscov(obj,Seq,dt)
            %constructTraj_lscov Construct the trajectory
            %   Seq: 1 x N, state sequence
            %   dt: scalar, time difference
            %   -----------------------------------------
            %   DataOutput: DP x N, expected position data
            %   SigmaOut: DP x DP x N, expected position covariance
            
            N = length(Seq);
            MuQ = reshape(obj.Mu(:,Seq), obj.D * N, 1);
            SigmaQ = ( kron(ones(N,1), eye(obj.D)) * reshape(obj.Sigma(:,:,Seq), obj.D, obj.D*N))...
                .* kron(eye(N), ones(obj.D));
            % % Or
            % SigmaQ = zeros(obj.N*D);
            % for t=1:N
            % 	id = (t-1)*obj.D+1:t*obj.D;
            % 	%MuQ(id) = obj.Mu(:,Seq(t));
            % 	SigmaQ(id,id) = obj.Sigma(:,:,Seq(t));
            % end
            PHI1 = obj.constructPhi1(N,dt);
            % % Least squares via lscov Matlab function
            [xhat,~,~,S] = lscov(PHI1, MuQ, SigmaQ, 'chol'); % Retrieval of data with weighted least squares solution
            DataOut = reshape(xhat, obj.DP, N); % Reshape data for plotting
            % % Least squares computation method 2 (most readable but not optimized)
            % PHIinvSigmaQ = PHI1' / SigmaQ;
            % Rq = PHIinvSigmaQ * PHI1;
            % rq = PHIinvSigmaQ * MuQ;
            % xhat = Rq \ rq; % Can also be computed with c = lscov(Rq, rq)
            % size(zeta)
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
    end
    
    methods (Access = protected)
        %% Dynamics functions
        [Phi,Phi1,Phi0] = constructPhi(obj,N,M,dt);
        [Phi1,Phi0] = constructPhi1(obj,N,dt);
    end
end

