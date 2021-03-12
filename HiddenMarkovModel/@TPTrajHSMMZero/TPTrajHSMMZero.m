classdef TPTrajHSMMZero < TrajHSMMZero
    %TPTrajHSMMZero Task-parameterzed trajectory hidden semi-Markov model
    %   Initialization with dim. of position, order of derivation, num. of states, and num. of frames.
    %   It is designed specifically for robot PbD applications.
    %
    %   Haopeng Hu
    %   2021.01.27
    %   All rights reserved.
    %
    %   Notations:
    %   |   D:  Dim. of data (DP*DD)
    %   |   DP: Dim. of position
    %   |   DD: Order of derivation
    %   |   N:  Num. of data
    %   |   K:  Num. of kernels
    %   |   M: Num. of demos
    %   |   F: Num. of frames
    %   |   TPDemo struct:
    %   |   |   data: D x N, demo data
    %   |   |   A: D x D x F, orientation matrices
    %   |   |   b: D x F, position vectors
    %   |   |   TPData: D x F x N, demo data in each frame
    %
    %   Recommendation:
    %   1   obj = TPTrajHSMMZero(DP,DD,K);
    %   2   obj = obj.initHMM...
    %   3   obj = obj.initTrans...
    %   4   obj = obj.leanHMM...
    %   5   [~,s] = obj.reconstructStSeq_...
    %   6   [Data] = obj.constructTraj_...
    
    properties
        F;              % Integer, num. of frames
        Mus;         % DP*DD x F x K, Mu in each frame (centers)
        Sigmas;     % DP*DD x DP*DD x F x K, Sigma in each frame (covariances)
    end
    
    methods
        function obj = TPTrajHSMMZero(DP,DD,K,F,logFlag)
            %TPTrajHSMMZero Initialization with dim. of position, order of derivation, num. of states, and num. of frames.
            %   DP: Integer, DP
            %   DD: Integer, DD
            %   K: Integer, K
            %   F:  Integer, F
            %   logFlag: Boolean, true for using lognormal duration
            %   distribution (default: false)
            DP = max([2, round(DP)]);
            DD = max([1, round(DD)]);
            K = max([1, round(K)]);
            F = max([1, round(F)]);
            if nargin < 5
                logFlag = false;
            end
            obj = obj@TrajHSMMZero(DP,DD,K,logFlag);
            obj.F = F;
            obj.Mus = zeros(DP*DD,F,K);
            obj.Sigmas = zeros(DP*DD, DP*DD, F, K);
        end
        
        function obj = initHMMKmeans(obj,TPDemos)
            %initHMMKMeans Init. the param. of HMM by KMeans agorithm.
            %   TPDemos: 1 x M TPDemo struct array
            diagRegularizationFactor = 1E-4;    %Optional regularization term
            Data = obj.dataFlattening(TPDemos,1);
            [idList,tmpMu] = obj.kMeans(Data);    % K-Means clustering
            tmpSigma = zeros(obj.D*obj.F, obj.D*obj.F, obj.K);
            for i = 1:obj.K
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                tmpSigma(:,:,i) = cov([Data(:,idtmp),Data(:,idtmp)]')+ eye(size(Data,1))*diagRegularizationFactor;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
            %Reshape GMM parameters into tensor data
            for m=1:obj.F
                for i=1:obj.K
                    obj.Mus(:,m,i) = tmpMu((m-1)*obj.D+1:m*obj.D,i);
                    obj.Sigmas(:,:,m,i) = tmpSigma((m-1)*obj.D+1:m*obj.D,(m-1)*obj.D+1:m*obj.D,i);
                end
            end
        end
        
        function [obj] = initHMMKmeansPosition(obj,TPDemos)
            %initHMMKmeansPosition Init. the param. of HMM by KMeans agorithm.
            %   TPDemos: 1 x M TPDemo struct array
            Data = obj.dataFlattening(TPDemos);
            [tmpMu, tmpSigma] = obj.init_tensor_kmeans(Data(1:obj.DP,:,:));
            obj.Mus(1:obj.DP, :, :) = tmpMu;
            obj.Sigmas(1:obj.DP, 1:obj.DP, :, :) = tmpSigma;
            obj.Sigmas(obj.DP+1:obj.D, obj.DP+1:obj.D, :, :) = ...
                obj.params_diagRegFact_KMeans*...
                repmat(eye(obj.D - obj.DP), [1,1,obj.F,obj.K]);
        end
        
        function obj = learnHMM(obj,Demos)
            %learnHMM	Learn the TP-HMM by EM algorithm
            %   TPDemos: 1 x M TPDemo struct array, demo data
            s.Data = []; M = length(Demos);
            s = repmat(s,[1,M]);
            for i = 1:M
                s(i).Data = Demos(i).TPData;
                s(i).nbData = size(Demos(i).TPData,3);
            end
            [obj, H] = obj.EM_tensorHMM(s);
            % Removal of self-transition and normalization
            obj.Trans = obj.Trans - diag(diag(obj.Trans)) + eye(obj.K)*realmin;
            obj.Trans = obj.Trans ./ repmat(sum(obj.Trans,2),1,obj.K);
            % Re-Estimation of transition param.
            % State duration storation
            st.d = [];
            st = repmat(st,[1,obj.K]);
            [~,hmax] = max(H);
            currState = hmax(1);
            if obj.logFlag
                % Log normal distribution
                cnt = 1;
                for t=1:length(hmax)
                    if (hmax(t)==currState)
                        cnt = cnt+1;
                    else
                        st(currState).d = [st(currState).d log(cnt)];
                        cnt = 1;
                        currState = hmax(t);
                    end
                end
                st(currState).d = [st(currState).d log(cnt)];
            else
                % Normal distribution
                cnt = 1;
                for t=1:length(hmax)
                    if (hmax(t)==currState)
                        cnt = cnt+1;
                    else
                        st(currState).d = [st(currState).d cnt];
                        cnt = 1;
                        currState = hmax(t);
                    end
                end
                st(currState).d = [st(currState).d cnt];
            end
            % Compute state duration as Gaussian distribution
            for i=1:obj.K
                obj.MuPd(1,i) = mean(st(i).d);
                obj.SigmaPd(1,1,i) = cov(st(i).d) + obj.minSigmaPd;
            end
        end
        
        %% Trajectory generation
        [DataOut, SigmaOut, obj] = constructTraj(obj,seq,frames);
        [DataOut, SigmaOut, obj] = constructTraj_lscov(obj,seq,frames);
        %% LQT
        [trajOut,u] = TPLQTIterative(obj, frames, seq);
        %% Demos regulation
        [Demo, frames] = TPDemoConstruct(obj,data,A,b);
        [Demo, frames] = TPDemoConstruct_Dynamic(obj,data,A,b);
    end
    
    methods (Access = protected)
        % Auxiliary func.
        [Data] = dataFlattening(obj,TPDemos,mode);
        [obj, GAMMA, GAMMA2] = EM_tensorHMM(obj, s);
        [Lik, GAMMA, GAMMA0] = computeGamma(obj, Data);
        [Mu,Sigma,obj] = prodLinearTransformedGauss(obj,frames);
        [Mu, Sigma] = init_tensor_kmeans(obj, Data);
    end
    
    methods (Access = public, Hidden = true)
        % The unsupported functions
        obj = initHMMKbins(obj,Demos);
    end
end

