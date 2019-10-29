classdef GMMOne
    %GMMOne The most simple Gaussian Mixture Model class
    %   You must assign num. of kernels initially.
    %   Notations:
    %   |   D: dimension of data
    %   |   K: num. of Gaussians
    %   |   M: num. of demos
    %   |   N: num. of data
    
    %   Haopeng Hu
    %   2019.10.29
    %   All rights reserved
    
    properties (Access = public)
        nKernel;        % K, number of Guassian kernels (states)
        nVar;           % D, number/Dimension of variables
        Mu;             % D x K, Gaussian means
        Sigma;          % D x D x K, Covariances
        Prior;          % 1 x K, Priors
    end
    
    properties (Access = protected)
        params_nbMinSteps = 5;              %Minimum number of iterations allowed
        params_nbMaxSteps = 100;            %Maximum number of iterations allowed
        params_maxDiffLL = 1E-4;            %Likelihood increase threshold to stop the algorithm
        params_diagRegFact = 1E-4;          %Regularization term is optional
        params_diagRegFact_KMeans = 1E-2;   %Regularization term for K-Means is optional
        params_diagRegFact_Cluster = 1E-8;  %Regularization term for clustering is optional
        params_updateComp = ones(3,1);      %pi,Mu,Sigma
    end
    
    methods (Access = public)
        function obj = GMMOne(nKernel,nVar)
            %GMMOne Assign num. of kernels and dimension of data
            %   nKernel: integer, the num. of Gaussians
            %   nVar: integer, the dimension of data
            obj.nKernel = nKernel;
            obj.nVar = nVar;
            obj.Mu = zeros(nVar,nKernel);
            obj.Sigma = zeros(nVar,nVar,nKernel);
            obj.Prior = zeros(1,nKernel);
        end
        
        function obj = initGMMKMeans(obj,Data)
            %initGMMKMeans Init. the param. of GMM by K-Means algorithm
            %   Data: D x (N * M), data of all demos
            [idList,obj.Mu] = obj.kMeans(Data);  % K-Means clustering
            for i = 1:obj.nKernel
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                obj.Sigma(:,:,i) = cov([Data(:,idtmp);Data(:,idtmp)]);
                % Optional regularization term to avoid numerical instability
                obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(obj.nVar)*obj.params_diagRegFact_KMeans;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
        end
        
        function obj = initGMMTimeBased(obj,Data)
            %initGMMTimeBased Initialize the GMM before EM based on time
            %   Data: D x (N * M), data of all demonstrations
            TimingSep = linspace(min(Data(1,:)), max(Data(1,:)), obj.nKernel+1); % Note that the first row is time series
            TmpMu = zeros(obj.nVar,obj.nKernel);
            for i=1:obj.nKernel
                idtmp = find( Data(1,:)>=TimingSep(i) & Data(1,:)<TimingSep(i+1));
                obj.Prior(i) = length(idtmp);
                TmpMu(:,i) = mean(Data(:,idtmp),2);
                obj.Sigma(:,:,i) = cov(Data(:,idtmp)');
                %Optional regularization term to avoid numerical instability
                obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(nbVar)*obj.params_diagRegFact_Cluster;
            end
            obj.Prior = obj.Prior / sum(obj.Prior);
            obj.Mu = TmpMu;
        end
        
        function obj = learnGMM(obj,Data)
            %learnGMM Learn the GMM by EM algorithm
            [obj.Mu,obj.Sigma,obj.Prior] = EMGMMOne(obj,Data);
        end
    end
    
    methods (Access = protected)
        [idList, Mu] = kMeans(obj,Data);
        [Mu,Sigma,Prior,GAMMA2,LL] = EMGMMOne(obj,Data)
    end
end

