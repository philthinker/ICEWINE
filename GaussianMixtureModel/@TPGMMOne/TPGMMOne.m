classdef TPGMMOne < GMMOne
    %TPGMMOne Task-parameterized Gaussian Mixture Model
    %   Init. the model with num. of distri., dim. of data and num. of
    %   frames.
    %   Demo data are stored in a struct array with the 'data','A' and 'b'
    %   fields.
    %   Note that we assume A and b are TIME-INVARIANT. If you wanna GMR
    %   with time/decaying query sequence, you must use augmented A and b.
    %   Notations:
    %   |   N: num. of data
    %   |   K: num. of Gaussian distri.
    %   |   F: num. of frames
    %   |   M: num. of demos
    %   |   D: dim. of data
    %   |   Din: dim. of the query data
    %   |   Dout: dmo. of the expected data
    %   |   TPDemo struct:
    %   |   |   data: D x N, demo data
    %   |   |   A: D x D x F, orientation matrices
    %   |   |   b: D x F, position vectors
    %   |   |   TPData: D x F x N, demo data in each frame
    %
    %   Haopeng Hu
    %   2020.03.05
    %   All rights reserved
    %
    
    properties
        nFrame; % num. of frames, F
        Mus;    % D x F x K, Mu in each frame (centers)
        Sigmas; % D x D x F x K, Sigma in each frame (covariances)
    end
    
    properties (Access = protected)
        Pix;        % Intermediate variable in TP-EM
    end
    
    methods
        function obj = TPGMMOne(nKernel,nVar,nFrame)
            %TPGMMOne Init. the TP-GMM with K, D and F
            %   nKernel: integer, K
            %   nVar: integer, D
            %   nFrame: integer, F
            K = round(nKernel(1,1));
            D = round(nVar(1,1));
            F = round(nFrame(1,1));
            obj = obj@GMMOne(K,D);
            obj.nFrame = F;
            obj.Mus = zeros(D,F,K);
            obj.Sigmas = zeros(D,D,F,K);
            obj.Pix = [];
        end
        
        function obj = initGMMKMeans(obj,TPDemos)
            %initGMMKMeans Init. the TP-GMM by K-Means algorithm. Note that
            %the func. DO NOT neglect the possible time/decaying term.
            %   TPDemos: 1 x M TPDemo struct array
            diagRegularizationFactor = 1E-4;    %Optional regularization term
            Data = obj.dataFlattening(TPDemos,1);
            [idList,tmpMu] = obj.kMeans(Data);    % K-Means clustering
            tmpSigma = zeros(obj.nVar*obj.nFrame, obj.nVar*obj.nFrame, obj.nKernel);
            for i = 1:obj.nKernel
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                tmpSigma(:,:,i) = cov([Data(:,idtmp),Data(:,idtmp)]')+ eye(size(Data,1))*diagRegularizationFactor;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
            %Reshape GMM parameters into tensor data
            for m=1:obj.nFrame
                for i=1:obj.nKernel
                    obj.Mus(:,m,i) = tmpMu((m-1)*obj.nVar+1:m*obj.nVar,i);
                    obj.Sigmas(:,:,m,i) = tmpSigma((m-1)*obj.nVar+1:m*obj.nVar,(m-1)*obj.nVar+1:m*obj.nVar,i);
                end
            end
        end
        
        function obj = initGMMTimeBased(obj,TPDemos)
            %initGMMTimeBased Init. the TP-GMM based on time sequence. We
            %always assume the time/decaying term is the first entry of the
            %demod data vector
            %   TPDemos: 1 x M TPDemo struct array, demo data
            diagRegularizationFactor = 1E-4; %Optional regularization term
            Data = obj.dataFlattening(TPDemos,1);
            tmpMu = zeros(obj.nFrame*obj.nVar, obj.nKernel);
            tmpSigma = zeros(obj.nVar*obj.nFrame, obj.nVar*obj.nFrame, obj.nKernel);
            %%%% Time based centers assignment %%%%
            TimingSep = linspace(min(Data(1,:)), max(Data(1,:)), obj.nKernel+1);
            for i = 1:obj.nKernel
                idtmp = find(Data(1,:)>=TimingSep(i) & Data(1,:)<TimingSep(i+1));
                obj.Prior(i) = length(idtmp);
                tmpMu(:,i) = mean(Data(:,idtmp),2);
                tmpSigma(:,:,i) = cov([Data(:,idtmp),Data(:,idtmp)]')+ eye(size(Data,1))*diagRegularizationFactor;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.Prior = obj.Prior/sum(obj.Prior);
            %%%% Reshape GMM parameters into tensor data %%%%
            for m=1:obj.nFrame
                for i=1:obj.nKernel
                    obj.Mus(:,m,i) = tmpMu((m-1)*obj.nVar+1:m*obj.nVar,i);
                    obj.Sigmas(:,:,m,i) = tmpSigma((m-1)*obj.nVar+1:m*obj.nVar,(m-1)*obj.nVar+1:m*obj.nVar,i);
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function [obj,GAMMA0,GAMMA2] = learnGMM(obj,TPDemos)
            %learnGMM Learn the TP-GMM by EM algorithm
            %   TPDemos: 1 x M TPDemo struct array, demo data
            Data = obj.dataFlattening(TPDemos,0);
            [obj,GAMMA0,GAMMA2] = obj.EMTPGMM(Data);
        end
        
        function [expData,expSigma,expGMR] = GMR(obj,query,qFrames)
            %GMR Gaussian mixture regression given query frames
            %   query: 1 x N, query sequence
            %   qFrames: 1  x F struct array, query frames
            %   |   A: D x D, orientation matrix
            %   |   b: D x 1, position vector
            %   expData: Dout x N, expected data in world frame
            %   expSigma: Dout x Dout x N, exptected covariances in world frame
            %   expGMR: 1 x F struct array, expected data and covariances in each frame
            %   |   A: D x D, orientation matrix
            %   |   b: D x 1, position vector
            %   |   data: Dout x N, expected data in one frame
            %   |   Sigma: Dout x Dout x N, expected covariances in one frame
            
            %%%% Init. the global param. & var. %%%%
            Dout = length(obj.Dataout);
            Din = length(obj.Datain);
            N = size(query,2);
            K = obj.nKernel;
            D = obj.nVar;
            F = obj.nFrame;
            expData = zeros(Dout,N); expSigma = zeros(Dout,Dout,N);
            expGMR = []; expGMR.A = eye(D); expGMR.b = zeros(1,D); 
            expGMR.data = expData; expGMR.Sigma = expSigma;
            expGMR = repmat(expGMR,[1,F]);
            for i = 1:F
                expGMR(i).A = qFrames(i).A;
                expGMR(i).b = qFrames(i).b;
            end
            MuGMR = zeros(Dout, N, F);             % Dout x N x F
            SigmaGMR = zeros(Dout, Dout, N, F);    % Dout x Dout x N x F
            %%%% TP-GMR %%%%
            H = zeros(K,N);
            for m=1:F
                %%%% Compute activation weights %%%%
                for i=1:K
                    H(i,:) = obj.Prior(i) * obj.GaussPDF(query, obj.Mus(obj.Datain,m,i), obj.Sigmas(obj.Datain,obj.Datain,m,i));
                end
                H = H ./ (repmat(sum(H),K,1)+realmin);
                MuTmp = zeros(Dout,K); 
                for t=1:N
                    %%%% Compute conditional means %%%%
                    for i=1:K
                        MuTmp(:,i) = obj.Mus(obj.Dataout,m,i) + ...
                            obj.Sigmas(obj.Dataout,obj.Datain,m,i) / obj.Sigmas(obj.Datain,obj.Datain,m,i) * (query(:,t) - obj.Mus(obj.Datain,m,i));
                        MuGMR(:,t,m) = MuGMR(:,t,m) + H(i,t) * MuTmp(:,i);
                    end
                    %%%% Compute conditional covariances %%%%
                    for i=1:K
                        SigmaTmp = obj.Sigmas(obj.Dataout,obj.Dataout,m,i) - ...
                            obj.Sigmas(obj.Dataout,obj.Datain,m,i) / obj.Sigmas(obj.Datain,obj.Datain,m,i) * obj.Sigmas(obj.Datain,obj.Dataout,m,i);
                        SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
                    end
                    SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) - MuGMR(:,t,m) * MuGMR(:,t,m)' + eye(length(obj.Dataout)) * obj.params_diagRegFact;   % 1E-8
                end
            end
            %%%% Compute the reproduction for the given query frames %%%%
            MuTmp = zeros(Dout, N, F);
            SigmaTmp = zeros(Dout, Dout, N, F);
            %%%% Linear transformation of the retrieved Gaussians to the world frame %%%%
            %%%% \xi = AX + b %%%%
            for m=1:F
                MuTmp(:,:,m) = qFrames(m).A(Din+1:end,Din+1:end) * MuGMR(:,:,m) + repmat(qFrames(m).b(Din+1:end),1,N);
                for t=1:N
                    SigmaTmp(:,:,t,m) = qFrames(m).A(Din+1:end,Din+1:end) * SigmaGMR(:,:,t,m) * qFrames(m).A(Din+1:end,Din+1:end)';
                end
            end
            %%%% Product of Gaussians (fusion of information from the different coordinate systems) %%%%
            for t=1:N
                SigmaP = zeros(length(obj.Dataout));
                MuP = zeros(length(obj.Dataout), 1);
                for m=1:F
                    SigmaP = SigmaP + inv(SigmaTmp(:,:,t,m));
                    MuP = MuP + SigmaTmp(:,:,t,m) \ MuTmp(:,t,m);
                end
                expSigma(:,:,t) = inv(SigmaP);
                expData(:,t) = expSigma(:,:,t) * MuP;
            end
            for i = 1:F
                expGMR(i).data = MuGMR(:,:,i);
                expGMR(i).Sigma = SigmaGMR(:,:,:,i);
            end
        end
        
    end
    
    methods (Access = public)
        % Auxiliary func.
        [Data] = dataFlattening(obj,TPDemos,mode);
        [Demo] = TPDemoConstruct(obj,Data,A,b,tmpTime);
    end
    
    methods (Access = protected)
        [Lik, GAMMA, GAMMA0] = computeTPGamma(obj,Data);
        [obj, GAMMA0, GAMMA2] = EMTPGMM(obj,Data);
    end

end

