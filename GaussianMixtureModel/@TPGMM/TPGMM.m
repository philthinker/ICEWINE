classdef TPGMM < GMMZero
    %TPGMM Task-parameterized Gaussian Mixture Model
    %   TP-GMM designed for PbD problem specifically.
    %   You must assign num. of kernels initially. Demonstration data must
    %   be aligned to vectors in cells.
    %   Notations:
    %       D: nVar, dimension of data without time/decay term
    %       K: nKernel, num. of Gaussians
    %       F: nFrame, num. of coordinate frames
    %       N: num. of data in each demo
    %       M: num. of demos
    %       TP-Demo struct:
    %           A: D x D x F, orientation matrix
    %           b: D x F, position
    %           data: N x D, original data in world frame
    %           TPData: D x F x N, data in each frame
    
    %   Haopeng Hu
    %   2019.10.23
    %   All rights reserved
    
    properties (Access = public)
        nFrame;     % Num. of frames
        A;          % D x D x F, \xi = A\mu + b
        b;          % D x F
        Mus;        % D x F x K, the Mu in each frame
        Sigmas;     % D x D x F x K, the Sigma in each frame
        Pix;        % Intermediate variable in TP-EM
    end
    
    
    methods
        function obj = TPGMM(nKernel,nVar,nFrame)
            %TPGMM Specify the num. of Gaussians and dimension of variable
            %   nKernel: integer, the num. of Gaussians
            %   nVar: Integer, the dimension of variable, DO NOT assign
            %   time/decay term to it!
            obj = obj@GMMZero(nKernel,nVar,1e-3); % The 1st column of data is time/decay term in GMMZero. But here we DO NOT need it.
            obj.nFrame = floor(nFrame);
            obj.A = repmat(eye(nVar),[1,1,obj.nFrame]);
            obj.b = zeros(nVar,obj.nFrame);    % Note that they are column vectors
            obj.Mus = zeros(obj.nVar,obj.nFrame,obj.nKernel);
            obj.Sigmas = repmat(zeros(nVar),[1,1,nFrame,nKernel]);
            obj.Pix = [];
        end
        
        function [obj] = initGMMKMeans(obj,Demos)
            %initGMMKMeans Init. the TPGMM by K-means algorithm
            %   Demos: 1 x M cell, TPDemo struct
            diagRegularizationFactor = 1E-4;    %Optional regularization term
            Data = obj.tpDataRegulate(Demos,true);
            [idList,Mu] = obj.kMeans(Data);    % K-Means clustering
            Sigma = zeros(obj.nVar*obj.nFrame, obj.nVar*obj.nFrame, obj.nKernel);
            for i = 1:obj.nKernel
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                Sigma(:,:,i) = cov([Data(:,idtmp),Data(:,idtmp)]')+ eye(size(Data,1))*diagRegularizationFactor;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
            %Reshape GMM parameters into tensor data
            for m=1:obj.nFrame
                for i=1:obj.nKernel
                    obj.Mus(:,m,i) = Mu((m-1)*obj.nVar+1:m*obj.nVar,i);
                    obj.Sigmas(:,:,m,i) = Sigma((m-1)*obj.nVar+1:m*obj.nVar,(m-1)*obj.nVar+1:m*obj.nVar,i);
                end
            end
        end
        
        function obj = learnGMM(obj,Demos)
            %learnGMM Learn the TPGMM by EM algorithm
            %   Demos: 1 x M cell, TPDemo struct
            [Data, obj.A, obj.b] = obj.tpDataRegulate(Demos);
            obj = obj.EMTPGMM(Data);
        end
        
        function [Mu,Sigma] = productTPGMM(obj,Demo)
            %productTPGMM Compute the product of Gaussians for a task-parametrized model
            %   Demo: TP-Demo struct
            %   Mu: D x K
            %   Sigma: D x D x K
            Mu = zeros(obj.nVar,obj.nKernel);
            Sigma = zeros(obj.nVar,obj.nVar,obj.nKernel);
            for i=1:obj.nKernel
                % Reallocating
                SigmaTmp = zeros(obj.nVar);
                MuTmp = zeros(obj.nVar,1);
                % Product of Gaussians
                for m=1:obj.nFrame
                    tmpA = Demo.A(:,:,m);
                    tmpb = Demo.b(:,m);
                    MuP = tmpA * obj.Mus(:,m,i) + tmpb;
                    SigmaP = tmpA * obj.Sigmas(:,:,m,i) * tmpA';
                    SigmaTmp = SigmaTmp + inv(SigmaP);
                    MuTmp = MuTmp + SigmaP\MuP;
                end
                Sigma(:,:,i) = inv(SigmaTmp);
                Mu(:,i) = Sigma(:,:,i) * MuTmp;
            end
        end
    end
    
    methods (Access = public)
        function demo = frameTransAb(obj,demo0,A,b)
            %frameTransAb Transform the demo from world frame to another
            %frame specified by A and b
            %   demo0: N x D, the original demo
            %   A: D x D, SO(D) matrix
            %   b: D x 1, position of the query frame
            demo = demo0';  % Note that N x D is friendly for engineering but D x N is friendly for research
            N = size(demo,2);
            for i = 1:N
                demo(:,i) = A\(demo(:,i)-b);
            end
            demo = demo';
        end
        [demo,TPData] = dataReconstruct(obj,A,b,data0);
        [Data,A,b] = tpDataRegulate(obj,Demos,kMeans);
    end
    
    methods (Access = protected)
        [Lik, GAMMA, GAMMA0] = computeTPGamma(obj,Data);
        [obj, GAMMA0, GAMMA2] = EMTPGMM(obj,Data);
        [idList, Mu] = kMeans(obj,Data);
    end
end

