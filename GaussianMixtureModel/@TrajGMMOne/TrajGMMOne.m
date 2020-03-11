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
    %   |   |   data: DPos x N, demo data
    %   |   |   A: D x D x F, orientation matrices
    %   |   |   b: D x F, position vectors
    %   |   |   TPData: DPos x F x N, demo data in each frame
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
        param_x_amplifier = 1E2;
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
            %   If obj.tpFlag == false:
            %   Demos: 1 x M struct array:
            %   |   data: DPos x N, demo data
            %   dt: scalar, time difference
            %   Data: D+1 x N*m, demo data with time sequence
            %   If obj.tpFlag == true:
            %   Demos: 1 x M TPDemo struct array
            if obj.tpFlag
                % Use Mus and Sigmas
                % Developping ...
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            else
                % Use Mu and Sigma
                [Data,Ns] = obj.dataFlattening_dyna(Demos,dt);
                M = length(Demos);
                TimingSeq = zeros(1,size(Data,2)); tmpIndex = 1;
                for m = 1:M
                    TimingSeq(tmpIndex:tmpIndex+Ns(m)-1) = (1:Ns(m));
                    tmpIndex = tmpIndex + Ns(m);
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
        
        function [obj,Data] = initGMMKMeans(obj,Demos,dt)
            %initGMMKMeans Init. the GMM by K-Means algorithm
            %   If obj.tpFlag == false
            %   Demos: 1 x M struct array:
            %   |   data: DPos x N, demo data
            %   dt: scalar, time difference
            %   Data: D x N*M, demo data in matrix form
            %   If obj.tpFlag == true
            %   Demos: 1 x M TPDemo struct array
            if obj.tpFlag
                % Use Mus and Sigmas
                
            else
                % Use Mu and Sigma
                Data = obj.dataFlattening_dyna(Demos,dt);
                obj = obj.initGMMKMeans@GMMOne(Data);
            end
        end
        
        function [obj,Data] = initGMMKMeans2EM_Pos(obj,TPDemos,dt)
            %initGMMKMeans_Pos Init. the GMM by K-Means and EM based on pos. demo data only
            %   TPDemos: 1 x M, TPDemo struct array
            %   dt: scalar, time difference
            if obj.tpFlag
                tmpModel = TPGMMOne(obj.nKernel,obj.nVar/obj.nDiff,obj.nFrame);
                tmpModel = tmpModel.initGMMKMeans(TPDemos);
                [~,~,GAMMA2] = tmpModel.learnGMM(TPDemos);
                Data = obj.dataFlattening_dyna(TPDemos,dt);
                obj.Prior = tmpModel.Prior;
                for i = 1:obj.nKernel
                    for j = 1:obj.nFrame
                        DataTmp = squeeze(Data(:,j,:));
                        obj.Mus(:,j,i) = DataTmp * GAMMA2(i,:)';
                        DataTmp = DataTmp - repmat(obj.Mus(:,j,i),1,size(Data,3)); % N*M
                        obj.Sigmas(:,:,j,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp';
                    end
                end
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
        
        function [expData, expSigma] = reproduct(obj,query,dt,LSMode)
            %reproduct Reproduction 
            %   query: 1 x N, the query state sequence
            %   dt: scalar, time difference
            %   LSMode: integer, the computation method of LS
            %   |   0: Using MATLAB func. lscov (Default)
            %   |   1: Most readable but not optimized method
            %   |   2: Using Cholesky and QR decompositions
            %   expData: DPos x N, expected data
            %   expSigma: DPos x DPos x N, expected covairances
            if nargin < 4
                LSMode = 0;
            end
            if obj.tpFlag
                % TP-Traj-GMM
            else
                % Ordinary Traj-GMM
                % Create single Gaussian N(MuQ,SigmaQ) based on query sequence q
                N = size(query,2);
                D = obj.nVar;
                DPos = obj.nVar/obj.nDiff;
                Phi1 = obj.constructPhi1(N,dt);
                MuQ = reshape(obj.Mu(:,query), D*N, 1);
                SigmaQ = zeros(D*N);
                for t=1:N
                    id = ((t-1)*D+1:t*D);
                    SigmaQ(id,id) = obj.Sigma(:,:,query(t));
                end
                if LSMode == 1
                    % Most readable but not optimized method
                    PHIinvSigmaQ = obj.Phi1' / SigmaQ;
                    Rq = PHIinvSigmaQ * (Phi1);
                    rq = PHIinvSigmaQ * MuQ;
                    zeta = Rq \ rq;             % Can also be computed with c = lscov(Rq, rq)
                    expData = reshape(zeta, DPos, N);  % Reshape data for plotting
                    %Covariance Matrix computation of ordinary least squares estimate
                    mse =  (MuQ'*SigmaQ\MuQ - rq'*Rq\rq) ./ ((D-DPos)*N);
                    S = Rq\mse;
                elseif LSMode == 2
                    % Using Cholesky and QR decompositions
                    T = chol(SigmaQ)'; %SigmaQ=T*T'
                    TA = T \ Phi1;
                    TMuQ = T \ MuQ;
                    [Q, R, perm] = qr(TA,0); %obj.Phi1(:,perm)=Q*R
                    z = Q' * TMuQ;
                    zeta = zeros(N * DPos,1);
                    zeta(perm,:) = R \ z; %zeta=(TA'*TA)\(TA'*TMuQ)
                    expData = reshape(zeta, DPos, N); %Reshape data for plotting
                    %Covariance Matrix computation of ordinary least squares estimate
                    err = TMuQ - Q*z;
                    mse = err'*err ./ (D * N - DPos * N);
                    Rinv = R \ eye(DPos * N);
                    S(perm,perm) = Rinv*Rinv' .* mse;
                else
                    % Using MATLAB func. lscov (Default)
                    [zeta,~,~,S] = lscov(Phi1, MuQ, SigmaQ, 'chol'); %Retrieval of data with weighted least squares solution
                    expData = reshape(zeta, DPos, N); %Reshape data for plotting
                end
                % Rebuild covariance by reshaping S
                expSigma = zeros(DPos,DPos,N);
                for t=1:N
                    id = ((t-1) * DPos+1 : t * DPos);
                    expSigma(:,:,t) = S(id,id) * N;
                end
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary func.
        [Data,Ns] = dataRegulate(obj,Demos);
        [dynaData,Ns,Phi] = dataFlattening_dyna(obj,Demos,dt);
        [query] = deriveQuery4Reprod(obj,Demo,N);
    end
    
    methods (Access = protected)
        [Phi] = constructPhi(obj,Demos,dt);
        [Phi1,Phi0] = constructPhi1(obj,N,dt);
    end
end

