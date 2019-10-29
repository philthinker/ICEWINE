classdef TPGMM < GMMZero
    %TPGMM Task-parameterized Gaussian Mixture Model
    %   TP-GMM designed for PbD problem specifically.
    %   You must assign num. of kernels initially. Demonstration data must
    %   be aligned to vectors in cells.
    %   Notations:
    %       D: nVar, dimension of data with/without time/decay term
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
        Mus;        % D x F x K, the Mu in each frame
        Sigmas;     % D x D x F x K, the Sigma in each frame
    end
    
    properties (Access = protected)
        Pix;        % Intermediate variable in TP-EM
        Datain;     % The indices of query variable in GMR (default:1)
        Dataout;    % The indices of regression variable in GMR (defult:(2:end)')
    end
    
    methods
        function obj = TPGMM(nKernel,nVar,nFrame)
            %TPGMM Specify the num. of Gaussians and dimension of variable
            %   nKernel: integer, the num. of Gaussians
            %   nVar: Integer, the dimension of variable, DO NOT assign
            %   time/decay term to it!
            obj = obj@GMMZero(nKernel,nVar,1e-3); % The 1st column of data is time/decay term in GMMZero. But here we DO NOT need it.
            obj.nFrame = floor(nFrame);
            obj.Mus = zeros(obj.nVar,obj.nFrame,obj.nKernel);
            obj.Sigmas = repmat(zeros(nVar),[1,1,nFrame,nKernel]);
            obj.Pix = [];
            obj.Datain = 1;
            obj.Dataout = (2:obj.nVar)';
        end
        
        function [obj] = initGMMKMeans(obj,Demos)
            %initGMMKMeans Init. the TPGMM by K-means algorithm
            %   Demos: 1 x M cell, TP-Demo struct
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
        
        function [obj] = initGMMTimeBased(obj,Demos)
            %initGMMTimeBased Init. the GMM w.r.t. time/decay term
            %   Demos: 1 x M cell, TP-Demo struct
            diagRegularizationFactor = 1E-4; %Optional regularization term
            Data = obj.tpDataRegulate(Demos,true);
            Mu = zeros(obj.nFrame*obj.nVar, obj.nKernel);
            Sigma = zeros(obj.nVar*obj.nFrame, obj.nVar*obj.nFrame, obj.nKernel);
            %%%% Time based centers assignment %%%%
            TimingSep = linspace(min(Data(1,:)), max(Data(1,:)), obj.nKernel+1);
            for i = 1:obj.nKernel
                idtmp = find(Data(1,:)>=TimingSep(i) & Data(1,:)<TimingSep(i+1));
                obj.Prior(i) = length(idtmp);
                Mu(:,i) = mean(Data(:,idtmp),2);
                Sigma(:,:,i) = cov([Data(:,idtmp),Data(:,idtmp)]')+ eye(size(Data,1))*diagRegularizationFactor;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
            [Data] = obj.tpDataRegulate(Demos);
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
        
        function [expData,expSigma,MuGMR,SigmaGMR] = GMR(obj,query,qFrames)
            %GMR Task-parameterized Gaussian mixture regression
            %   query: Din x N, the query variable  sequence
            %   expData: Dout x N, expected/regression data
            %   expSigma:  Dout x Dout x N, exptected covariances
            %   MuGMR: Dout x N x F, expected/regression data in each frame
            %   SigmaGMR: Dout x Dout x N x  F, expected covariances in each frame
            %   qFrames: struct('A',D x D,'b',D x 1) x obj.nFrame, the given pair of query frame
            Dout = length(obj.Dataout);
            Din = length(obj.Datain);
            nData = size(query,2);
            MuGMR = zeros(Dout, nData, obj.nFrame);             % Dout x N x F
            SigmaGMR = zeros(Dout, Dout, nData, obj.nFrame);    % Dout x Dout x N x F
            % TP-GMR
            H = zeros(obj.nKernel,nData);
            for m=1:obj.nFrame
                %Compute activation weights
                for i=1:obj.nKernel
                    H(i,:) = obj.Prior(i) * obj.GaussianPD(query, obj.Mus(obj.Datain,m,i), obj.Sigmas(obj.Datain,obj.Datain,m,i));
                end
                H = H ./ (repmat(sum(H),obj.nKernel,1)+realmin);
                MuTmp = zeros(Dout,obj.nKernel);
                for t=1:nData
                    %Compute conditional means
                    for i=1:obj.nKernel
                        MuTmp(:,i) = obj.Mus(obj.Dataout,m,i) + ...
                            obj.Sigmas(obj.Dataout,obj.Datain,m,i) / obj.Sigmas(obj.Datain,obj.Datain,m,i) * (query(:,t) - obj.Mus(obj.Datain,m,i));
                        MuGMR(:,t,m) = MuGMR(:,t,m) + H(i,t) * MuTmp(:,i);
                    end
                    %Compute conditional covariances
                    for i=1:obj.nKernel
                        SigmaTmp = obj.Sigmas(obj.Dataout,obj.Dataout,m,i) - ...
                            obj.Sigmas(obj.Dataout,obj.Datain,m,i) / obj.Sigmas(obj.Datain,obj.Datain,m,i) * obj.Sigmas(obj.Datain,obj.Dataout,m,i);
                        SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
                    end
                    SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) - MuGMR(:,t,m) * MuGMR(:,t,m)' + eye(length(obj.Dataout)) * obj.params_diagRegFact;   % 1E-8
                end
            end
            % Compute the reproduction for the given pair of query frame
            expData = zeros(Dout,nData);
            expSigma = zeros(Dout,Dout,nData);
            MuTmp = zeros(Dout, nData, obj.nFrame);
            SigmaTmp = zeros(Dout, Dout, nData, obj.nFrame);
            % Linear transformation of the retrieved Gaussians into the
            % world frame
            % \xi = AX + b
            for m=1:obj.nFrame
                MuTmp(:,:,m) = qFrames(m).A(Din+1:end,Din+1:end) * MuGMR(:,:,m) + repmat(qFrames(m).b(Din+1:end),1,nData);
                for t=1:nData
                    SigmaTmp(:,:,t,m) = qFrames(m).A(Din+1:end,Din+1:end) * SigmaGMR(:,:,t,m) * qFrames(m).A(Din+1:end,Din+1:end)';
                end
            end
            % Product of Gaussians (fusion of information from the different coordinate systems)
            for t=1:nData
                SigmaP = zeros(length(obj.Dataout));
                MuP = zeros(length(obj.Dataout), 1);
                for m=1:obj.nFrame
                    SigmaP = SigmaP + inv(SigmaTmp(:,:,t,m));
                    MuP = MuP + SigmaTmp(:,:,t,m) \ MuTmp(:,t,m);
                end
                expSigma(:,:,t) = inv(SigmaP);
                expData(:,t) = expSigma(:,:,t) * MuP;
            end
        end
        
    end
    
    methods (Access = public)
        [demo,TPData] = dataReconstruct(obj,A,b,data0);
        [Data] = tpDataRegulate(obj,Demos,kMeans);
        
        function [obj,Datain,Dataout] = setGMRIOIndex(obj, Datain, Dataout)
            %setGMRIOIndex Set the indices of data-in and data-out for GMR
            %   Datain: N1 x 1, indices of data-in (optional)
            %   Dataout: N2 x 1, indices of data-out (optional)
            %   Datain + Dataout = obj.nVar
            %   If you do not assign the arguments above, this method can
            %   be used to get the Datain and Dataout properties of obj
            if nargin < 2
                % Get the Datain and Dataout properties of obj
                Datain = obj.Datain;
                Dataout = obj.Dataout;
            else
                % Set the Datain and Dataout properties of obj
                obj.Datain = Datain;
                obj.Dataout = Dataout;
            end
        end
        
    end
    
    methods (Access = protected)
        [Lik, GAMMA, GAMMA0] = computeTPGamma(obj,Data);
        [obj, GAMMA0, GAMMA2] = EMTPGMM(obj,Data);
        [idList, Mu] = kMeans(obj,Data);
    end
end

