classdef HSMMZero
    %HSMMZero Hidden semi-Markov Model with Gaussian kernels
    %   Initialization with dim. and num. of states.
    %   It is designed specifically for robot PbD applications.
    %
    %   Haopeng Hu
    %   2021.01.01 Happy New Year!
    %   All rights reserved.
    %
    %   Notations:
    %   |   D:  Dim. of data
    %   |   N:  Num. of data
    %   |   K:  Num. of kernels
    %   |   M: Num. of demos
    %
    %   Recommendation:
    %   1   obj = HSMMZero(D,K);
    %   2   obj = obj.initHMM...
    %   3   obj = obj.initTrans...
    %   4   obj = obj.leanHMM...
    
    properties
        D;              % Integer, Dim. of states
        K;              % Integer, Num. of states
        Mu;           % D x K, Centers of Gaussians
        Sigma;       % D x D x K, Covariances of Gaussians
        Prior;         % 1 x K, Priors of GMM (unusable)
        StatePrior; % 1 x K, State priors
        Trans;        % K x K, Transition of states
        MuPd;       % 1 x K, Temporal centers of each state
        SigmaPd;   % 1 x 1 x K, Temporal variances of each state
        dt;             % Scalar, Time difference
        logFlag;     % Boolean, true for using log normal duration distribution
    end
    
    properties (Access = protected)
        params_diagRegFact_KMeans =1E-3;     % Regularization term for K-Means is optional
        params_diagRegFact = 1E-8;                  % Regularization term is optional
        params_nbMaxSteps = 99;                     % Maximum number of iterations allowed
        params_nbMinSteps = 5;                        % Minimum number of iterations allowed
        params_updateComp = ones(4,1);          % Mu, Sigma, Prior, Trans
        params_maxDiffLL = 1E-4;                      % Likelihood increase threshold to stop the algorithm
        minSigmaPd = 10;                                  % Minimum variance of state duration (regularization term)
    end
    
    methods
        function obj = HSMMZero(D,K,logFlag)
            %HSMMZero Initialization with dim. and num. of kernels
            %   D: Integer, dim. of states
            %   K: Integer, num. of Gaussian states
            %   logFlag: Boolean, true for using log normal duration
            %   distribution. (default: false)
            obj.D = ceil(D(1,1));
            obj.K = ceil(K(1,1));
            if nargin < 3
                logFlag = false;
            end
            obj.logFlag = logFlag;
            obj.Mu = zeros(obj.D, obj.K);
            obj.Sigma = zeros(obj.D, obj.D, obj.K);
            obj.Prior = ones(1,obj.K);
            obj.StatePrior = zeros(1, obj.K); obj.StatePrior(1) = 1;
            obj.Trans = eye(obj.K);
            obj.dt = 1e-3;
        end
        %% Model initialization and learning
        obj = initHMMKmeans(obj,Demos);
        obj = initHMMKbins(obj,Demos);      % Recommended
        obj = initTransUniform(obj);              % Recommended
        obj = initTransRandom(obj);
        obj = initTransLeftRight(obj,N);
        
        %% Param. estimation
        function [obj] = learnHMM(obj, Demos)
            %learnHMM Learn the HMM with EM algorithm
            %   Demos: 1 x M cells of D x N data, demos
            
            % EM algorithm
            [obj, H] = obj.EMHMMZero(Demos);
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
        
        %% HSMM Inference
        [h,s] = reconstructStSeq_StandardFW(obj, N);
        [h,s] = reconstructStSeq_FastFW(obj,N);
        [h,s] = reconstructStSeq_StochasticSamp(obj, N);
    end
    
    methods (Access = protected)
        %% General functions
        [Data,M,N,D] = dataRegulate(obj,Demos);
        [idList, Mu] = kMeans(obj,Data);
        [prob] = GaussPDF(obj,Data, Mu, Sigma);
        %% HSMM functions
        [obj,GAMMA2,LL] = EMHMMZero(obj,Demos);
        [Pd, NGen] = durationAssign_Uniform(obj,N,c);
    end
end

