classdef GMMOne
    %GMMOne The most simple Gaussian Mixture Model class
    %   You must assign num. of kernels initially.
    %   Notations:
    %   |   D:      dimension of data
    %   |   Din:    dimension of Datain in GMR
    %   |   Dout:   dimension of Dataout in GMR
    %   |   K:      num. of Gaussians
    %   |   M:      num. of demos
    %   |   N:      num. of data
    %
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
        
        Datain;                             %Indices of the query data in GMR
        Dataout;                            %Indices of the regression data in GMR
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
            obj.Datain = 1;
            obj.Dataout = (2:obj.nVar)';
        end
        
        function obj = initGMMKMeans(obj,Data)
            %initGMMKMeans Init. the param. of GMM by K-Means algorithm
            %   Data: D x (N * M), data of all demos
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
                obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(obj.nVar)*obj.params_diagRegFact_Cluster;
            end
            obj.Prior = obj.Prior / sum(obj.Prior);
            obj.Mu = TmpMu;
        end
        
        function obj = initGMMKBins(obj,Data,M,N)
            %initGMMKBins Initialize the GMM before EM by clustering an
            %ordered dataset into equal bins.
            %   Data: D x (N * M), data of all demonstrations
            %   M: integer, num. of demonstrations
            %   N: integer, num. of data in each demonstration
            
            %Delimit the cluster bins for the first demonstration
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
        
        function [obj,GAMMA2] = learnGMM(obj,Data)
            %learnGMM Learn the GMM by EM algorithm
            %   Data: D x (N * M), all the demonstration data.
            [obj,GAMMA2] = EMGMMOne(obj,Data);
        end
        
        function [expData, expSigma] = GMR(obj,query)
            %GMR Gaussian Mixture Regression
            %   query: 1 x N, the query series (time/decay term)
            [expData,expSigma] = obj.GMROne(query,obj.Datain,obj.Dataout);
        end
    end
    
    methods (Access = protected)
        [idList, Mu] = kMeans(obj,Data);
        [obj,GAMMA2,LL] = EMGMMOne(obj,Data);
        [expData,expSigma] = GMROne(obj,query,in,out);
        [prob] = GaussPDF(obj,Data, Mu, Sigma);
    end
    
    methods (Access = public)
        function [obj] = setGMRInOut(obj,datain,dataout)
            %setGMRInOut Set the Datain and Dataout property of the GMM
            %which are used in GMR
            %   datain: Din x 1, Datain
            %   dataout: Dout x 1, Dataout (optional)
            if nargin < 3
                tmpIndex = (1:obj.nVar)';
                dataout = setdiff(tmpIndex,datain);
            end
            if size(datain,1) + size(dataout,1) == obj.nVar
                obj.Datain = datain;
                obj.Dataout = dataout;
            end
        end
        
        function [Data] = dataRegulate(obj,Demos)
            %dataRegulate Regulate the data in Demos into one matrix
            %   Demos: 1 x M cell, the demos. Each cell contains one
            %   demonstration of D x N data
            M = size(Demos,2);
            tmpN = zeros(1,M);
            tmpIndex = 0;
            for i = 1:M
                tmpN(i) = size(Demos{i},2);
            end
            Data = zeros(obj.nVar,sum(tmpN));
            for i = 1:M
                for j = 1:tmpN(i)
                    Data(:,tmpIndex + j) = Demos{i}(1:obj.nVar,j);
                end
                tmpIndex = tmpIndex + tmpN(i);
            end
        end
        
        function [obj] = initGMMTimeBased_TmpTime(obj,Data,N)
            %initGMMTimeBased_TmpTime Init. param. of the GMM based on
            %temporary time sequence
            %   Data: D x (N * M): demo data
            %   N: integer, the num. of data in one demo. If the num. of
            %   data are not identical in each demo, you must resampling
            %   the demos.
            M = floor(size(Data,2)/N);
            Data = [repmat((1:N),[1,M]);Data];
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
end

