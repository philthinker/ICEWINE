classdef FWTPGMM < TPGMMOne
    %FWTPGMM Frame-relate weighted TP-GMM
    %   TP-GMM with frame weights for better extrapolition performance
    %   Notations:
    %   |   K:  num. of Gaussians
    %   |   D:  dim. of states
    %   |   F:  num. of frames
    %   
    %   Haopeng Hu
    %   2020.07.25
    %   All rights reserved
    
    properties
    end
    
    methods
        function obj = FWTPGMM(K,D,F)
            %FWTPGMM Init. the TP-GMM with K, D, F
            %   K: integer, num. of Gaussians
            %   D: integer, dim.
            %   F: interger, num. of frames
            obj = obj@TPGMMOne(K,D,F);
        end
        
        function [expData,expSigma,alpha] = GMR(obj,query,qFrames)
            %GMR Gaussian mixture regression given query frames
            %   query: 1 x N, query sequence
            %   qFrames: 1  x F struct array, query frames
            %   |   A: D x D, orientation matrix
            %   |   b: D x 1, position vector
            %   expData: Dout x N, expected data in world frame
            %   expSigma: Dout x Dout x N, exptected covariances in world frame
            %   alpha: 1 x N, weights
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
            %%%% Weights init. %%%%
            alpha = ones(F,N);    % Uniform distribution init
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
%                     SigmaTmp(:,:,t,m) = qFrames(m).A(Din+1:end,Din+1:end) * SigmaGMR(:,:,t,m) * qFrames(m).A(Din+1:end,Din+1:end)';
                    %%% Weights computation %%%
                    tmpWeigths = 0;
                    for f = 1:F
                        tmpWeigths = tmpWeigths + norm(SigmaGMR(:,:,t,f));
                    end
                    alpha(m,t) = norm(SigmaGMR(:,:,t,m))/tmpWeigths;
                    SigmaTmp(:,:,t,m) = (qFrames(m).A(Din+1:end,Din+1:end) * SigmaGMR(:,:,t,m) * qFrames(m).A(Din+1:end,Din+1:end)')*alpha(m,t);
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
end

