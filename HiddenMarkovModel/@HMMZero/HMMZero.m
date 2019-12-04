classdef HMMZero
    %HMMZero The simplest Hidden Markov Model. We assume that each state is
    %a Gaussian distribution.
    %   You must assign num. of kernels initially.
    %   Notations:
    %   |   D:      dimension of data
    %   |   K:      num. of states
    %   |   M:      num. of demos
    %   |   N:      num. of data
    %   |   A:      the transition matrix
    
    %   Haopeng Hu
    %   2019.11.27
    %   All rights reserved
    
    properties (Access = public)
        nKernel;        % Num. of states
        nVar;           % Dimension of data
        Mu;             % Centers
        Sigma;          % Covariances
        Prior;          % Priors
        StatePrior;     % State prior
        Trans;          % Transition matrix
    end
    
    properties (Access = protected)
        params_nbMinSteps = 5;              % Minimum number of iterations allowed
        params_nbMaxSteps = 88;             % Maximum number of iterations allowed
        params_maxDiffLL = 1E-4;            % Likelihood increase threshold to stop the algorithm
        params_diagRegFact = 1E-8;          % Regularization term is optional
        params_updateComp = ones(4,1);      % Mu,Sigma,Prior,A
        params_diagRegFact_KMeans = 1E-2;   % Regularization term for K-Means is optional
    end
    
    methods (Access = public)
        function obj = HMMZero(nKernel,nVar)
            %HMMZero Initialize the HMM with the num. of states
            %   nKernel: integer, num. of states
            %   nVar: integer, dim. of data
            obj.nKernel = floor(nKernel);
            obj.nVar = floor(nVar);
            obj.Mu = zeros(obj.nVar,obj.nKernel);
            obj.Sigma = repmat(eye(obj.nVar),[1,1,obj.nKernel]);
            obj.Prior = zeros(1,obj.nKernel);
            obj.StatePrior = zeros(1,obj.nKernel); obj.StatePrior(1) = 1;
            obj.Trans = eye(obj.nKernel);
        end
        
        function obj = initHMMKMeans(obj,Demos)
            %initHMMKMeans Init. the param. of HMM by K-Means algorithm
            %   Demos: 1 x M cell, demos
            %   Data: D x (N * M), data of all demos
            Data = obj.dataRegulate(Demos);
            [idList,obj.Mu] = obj.kMeans(Data);  % K-Means clustering
            for i = 1:obj.nKernel
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                obj.Sigma(:,:,i) = cov([Data(:,idtmp) Data(:,idtmp)]');
                % Optional regularization term to avoid numerical instability
                obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(obj.nVar)*obj.params_diagRegFact_KMeans;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
        end
        
        function obj = initHMMKbins(obj,Demos)
            %initHMMKbins Init. the param. of HMM by Kbins clustering
            %   Demos: 1 x M cell, demos
            [Data,M] = obj.dataRegulate(Demos);
            N = size(Demos{1},2);   %Delimit the cluster bins for the first demonstration
            tSep = round(linspace(0, N, obj.nKernel+1));
            
            %Compute statistics for each bin
            for i=1:obj.nKernel
                id=[];
                for n=1:M
                    id = [id (n-1)*N+[tSep(i)+1:tSep(i+1)]];
                end
                obj.Prior(i) = length(id);
                obj.Mu(:,i) = mean(Data(:,id),2);
                obj.Sigma(:,:,i) = cov(Data(:,id)') + eye(size(Data,1)) * obj.params_diagRegFact;
            end
            obj.Prior = obj.Prior / sum(obj.Prior);
        end
        
        function [obj,Trans,StatePrior] = initTrans(obj,mode,N)
            %initTrans Initialize transition matrix A
            %   mode: String, 'r' for random init., 'u' for uniform
            %   init.(default), 'l' for left-right init..
            if nargin < 2
                mode = 'u';
            end
            
            if strcmp(mode,'r')
                % Randm init.
                obj.Trans = rand(obj.nKernel,obj.nKernel);
                obj.Trans = obj.Trans./repmat(sum(obj.Trans,2),1,obj.nKernel);
                obj.StatePrior = rand(obj.nKernel,1);
                obj.StatePrior = obj.StatePrior/sum(obj.StatePrior);
            elseif strcmp(mode,'u')
                % Uniform init.
                obj.Trans = ones(obj.nKernel,obj.nKernel);
                obj.Trans = obj.Trans./repmat(sum(obj.Trans,2),1,obj.nKernel);
                obj.StatePrior = ones(obj.nKernel,1);
                obj.StatePrior = obj.StatePrior/sum(obj.StatePrior);
            elseif strcmp(mode,'l')
                % Left-Right init.
                if nargin >= 3
                    % N is necessary
                    obj.Trans = zeros(obj.nKernel);
                    for i = 1:obj.nKernel-1
                        obj.Trans(i,i) = 1 - obj.nKernel/N;
                        obj.Trans(i,i+1) = obj.nKernel/N;
                    end
                    obj.Trans(obj.nKernel,obj.nKernel) = 1.0;
                    obj.StatePrior = zeros(obj.nKernel,1);
                    obj.StatePrior(1) = 1;
                end
            end
            
            Trans = obj.Trans;
            StatePrior = obj.StatePrior;
        end
        
        function obj = learnHMM(obj,Demos)
            %learnHMM by EM algorithm
            %   Demos: 1 x M cell, the demos
            obj = obj.EMHMMZero(Demos);
        end
    end
    
    methods (Access = protected)
        [idList, Mu] = kMeans(obj,Data);
        [obj] = EMHMMZero(obj,Demos);
        [prob] = GaussPDF(obj,Data, Mu, Sigma);
        [Data,M,N,D] = dataRegulate(obj,Demos);
    end
    
end

