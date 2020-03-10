classdef TrajGMMOne < TPGMMOne
    %TrajGMMOne Trajectory-Gaussian mixture model
    %   It provides a reproduction solution besides GMR. Task-parameterized
    %   setting is also supported.
    %   Init. it with K, D, DD, F. For efficiency, when F == 1 we use
    %   obj.Mu instead of obj.Mus, so do obj.Sigma and obj.Sigmas.
    %   Notations:
    %   |   N: num. of data
    %   |   M: num. of demos
    %   |   K: num. of Gaussian distri.
    %   |   F: nun. of frames
    %   |   D: dim. of data
    %   |   DD: ord. of diff. (1 pos., 2 vel., 3 acc., 4 jer.)
    %   |   DPos: dim. of the pos. data
    %   |   TPDemo struct:
    %   |   |   data: D x N, demo data
    %   |   |   A: D x D x F, orientation matrices
    %   |   |   b: D x F, position vectors
    %   |   |   TPData: D x F x N, demo data in each frame
    %
    %   Haopeng Hu
    %   2020.03.07
    %   All rights reserved
    %
    
    properties (Access = public)
        nDiff;      % ord. of diff., DD
    end
    
    properties (Access = protected)
        tpFlag;     % boolean, true for TP-Traj-GMM
    end
    
    methods
        function obj = TrajGMMOne(nKernel,nPosVar,nDiff,nFrame)
            %TrajGMMOne Init. the Traj-GMM with K, DPos, DD, F.
            %   nKernel: integer, K
            %   nPosVar: integer, dim. of pos. var., D/DD
            %   nDiff: integer, DD
            %   nFrame: integer, F
            K = round(nKernel(1,1)); DPos = round(nPosVar(1,1));
            DD = round(nDiff(1,1)); F = round(nFrame(1,1));
            if DD >= 4  % Only pos., vel., acc. and jer. supported
                DD = 4;
            end
            obj = obj@TPGMMOne(K, DPos * DD, F);
            obj.tpFlag = F > 1;
            obj.nDiff = DD;
        end
        
        function [obj,Data] = initGMMTimeBased_TmpTime(obj,Demos,dt)
            %initGMMTimeBased_TmpTime Init. param. of the GMM based on
            %temporary time sequence
            %   Demos: 1 x M struct array:
            %   |   data: DPos x N, demo data
            %   dt: scalar, time difference
            %   Data: D+1 x N*m, demo data with time sequence
            if obj.tpFlag
                % Use Mus and Sigmas
            else
                % Use Mu and Sigma
                [Data,Ns] = obj.dataFlattening_dyna(Demos,dt);
                M = length(Demos);
                TimingSeq = zeros(1,size(Data,2)); tmpIndex = 1;
                for m = 1:M
                    TimingSeq(1:tmpIndex:tmpIndex+Ns(m)-1) = (1:Ns);
                end
                Data = [TimingSeq;Data];
                TimingSep = linspace(min(Data(1,:)), max(Data(1,:)), obj.nKernel+1); % Note that the first row is time series
                TmpMu = zeros((obj.nVar)+1,obj.nKernel);
                TmpSigma = zeros((obj.nVar)+1,(obj.nVar)+1,obj.nKernel);
                for i=1:obj.nKernel
                    idtmp = find( Data(1,:)>=TimingSep(i) & Data(1,:)<TimingSep(i+1));
                    obj.Prior(i) = length(idtmp);
                    TmpMu(:,i) = mean(Data(:,idtmp),2);
                    TmpSigma(:,:,i) = cov(Data(:,idtmp)');
                    %Optional regularization term to avoid numerical instability
                    TmpSigma(:,:,i) = TmpSigma(:,:,i) + eye((obj.nVar)+1)*obj.params_diagRegFact_Cluster;
                end
                obj.Prior = obj.Prior / sum(obj.Prior);
                obj.Mu = TmpMu(2:end,:);
                obj.Sigma = TmpSigma(2:end,2:end,:);
            end
        end
        
        function obj = learnGMM(obj,Demos,dt)
            %learnGMM Learn the GMM by EM algorithm
            %   Demos: 1 x M struct array:
            %   |   data: D x N, demo data
            %   dt: scalar, time difference
            if obj.tpFlag
                % TP-Traj-GMM
            else
                % Ordinary Traj-GMM
                Data = obj.dataFlattening_dyna(Demos,dt);
                obj = obj.learnGMM@GMMOne(Data);
            end
        end
        
        function [expData, expSigma] = reproduct(obj,query,LSMode)
            %reproduct Reproduction 
            %   query: 1 x N, the query state sequence
            %   LSMode: integer, the computation method of LS
            %   |   0: Using MATLAB func. lscov (Default)
            %   |   1: Most readable but not optimized method
            %   |   2: Using Cholesky and QR decompositions
            %   expData: DPos x N, expected data
            %   expSigma: DPos x DPos x N, expected covairances
            if nargin < 3
                LSMode = 0;
            end
            if obj.tpFlag
                % TP-Traj-GMM
            else
                % Ordinary Traj-GMM
                % Create single Gaussian N(MuQ,SigmaQ) based on query sequence q
                N = size(query,2);
                MuQ = reshape(obj.Mu(:,query), (obj.nVar)*N, 1);
                SigmaQ = zeros((obj.nVar)*N);
                for t=1:obj.nData
                    id = ((t-1)*(obj.nVar)+1:t*(obj.nVar));
                    SigmaQ(id,id) = obj.Sigma(:,:,query(t));
                end
                if LSMode == 1
                    % Most readable but not optimized method
                    PHIinvSigmaQ = obj.Phi1' / SigmaQ;
                    Rq = PHIinvSigmaQ * (obj.Phi1);
                    rq = PHIinvSigmaQ * MuQ;
                    zeta = Rq \ rq;             % Can also be computed with c = lscov(Rq, rq)
                    expData = reshape(zeta, model.nbVarPos, nbData);  % Reshape data for plotting
                    %Covariance Matrix computation of ordinary least squares estimate
                    mse =  (MuQ'*SigmaQ\MuQ - rq'*Rq\rq) ./ ((obj.nVar-obj.nVarPos)*(obj.nData));
                    S = Rq\mse;
                elseif LSMode == 2
                    % Using Cholesky and QR decompositions
                    T = chol(SigmaQ)'; %SigmaQ=T*T'
                    TA = T \ obj.Phi1;
                    TMuQ = T \ MuQ;
                    [Q, R, perm] = qr(TA,0); %obj.Phi1(:,perm)=Q*R
                    z = Q' * TMuQ;
                    zeta = zeros(obj.nData * obj.nVarPos,1);
                    zeta(perm,:) = R \ z; %zeta=(TA'*TA)\(TA'*TMuQ)
                    expData = reshape(zeta, obj.nVarPos, obj.nbData); %Reshape data for plotting
                    %Covariance Matrix computation of ordinary least squares estimate
                    err = TMuQ - Q*z;
                    mse = err'*err ./ ((obj.nVar) * (obj.nData) - (obj.nVarPos) * (obj.nData));
                    Rinv = R \ eye((obj.nVarPos) * (obj.nData));
                    S(perm,perm) = Rinv*Rinv' .* mse;
                else
                    % Using MATLAB func. lscov (Default)
                    [zeta,~,mse,S] = lscov(obj.Phi1, MuQ, SigmaQ, 'chol'); %Retrieval of data with weighted least squares solution
                    expData = reshape(zeta, obj.nVarPos, obj.nData); %Reshape data for plotting
                end
                % Rebuild covariance by reshaping S
                expSigma = zeros(obj.nVarPos,obj.nVarPos,obj.nData);
                for t=1:obj.nData
                    id = ((t-1) * (obj.nVarPos)+1 : t * (obj.nVarPos));
                    expSigma(:,:,t) = S(id,id) * obj.nData;
                end
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary func.
        [Data,Ns] = dataRegulate(obj,Demos);
        [dynaData,Ns,Phi] = dataFlattening_dyna(obj,Demos,dt);
    end
    
    methods (Access = protected)
        [Phi] = constructPhi(obj,Demos,dt);
        [Phi1,Phi0] = constructPhi1(obj,N,dt);
    end
end

