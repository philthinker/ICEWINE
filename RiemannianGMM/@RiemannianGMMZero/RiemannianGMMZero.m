classdef RiemannianGMMZero
    %RiemannianGMMZero Gaussian Mixture Model on Riemannian manifold for
    %Unit Quaternion space
    %   We assumes you use this class for modelling and generalization of
    %   orientation of robots
    %   
    %   Haopeng Hu
    %   2020.07.13 Good Luck!
    %   All rights reserved
    %
    %   Notations:
    %   |   D:      dimension of data in tangent space (Include query)
    %   |   Din:    dimension of Datain in GMR (1, dim. of time query)
    %   |   Dout:   dimension of Dataout in GMR (3, dim. of tangent space)
    %   |   DoutM: dimension of DataoutM in GMR (4, dim. of unit quaternion)
    %   |   K:      num. of Gaussians
    %   |   M:      num. of demos
    %   |   N:      num. of data
    %   Data struct: RiemannianData
    %   |   data:   Dout x N, data in the tangent space
    %   |   dataM:  DoutM x N, data in manifold
    %   |   queryIn:  Din x N, query data to be learned (For GMR)
    %   |   queryGMR:  Din x N_query, query data
    %   |   expMu
    %   |   expSigma
    
    properties
        nKernel;         % K, number of Guassian kernels (states)
        nVar;             % D, number/Dimension of variables in tangent space (May include query)
        nVarM;          % (DoutM+Din) == D+1, number/Dimension of variables on manifold (May include query)
        Mu;               % D x K, Gaussian means
        MuMan;        % (DoutM+Din) x K, Gaussian means on Riemannian manifold
        Sigma;           % D x D x K, Covariances
        Prior;              % 1 x K, Priors
        center = [0;1;0;0];                % Center of the manifold 
                                                  % (Default: [w; i; j; k] = [0; 1; 0; 0], i.e. Rotate the original frame for pi/2 around x-axis )
                                                  % Refer to this websit:
                                                  % https://eater.net/quaternions/video/intro
                                                  % for visualization.
    end
    
    properties (Access = protected)
        % Constant Parameters
        params_nbMinSteps = 5;                  %Minimum number of iterations allowed
        params_nbMaxSteps = 100;              %Maximum number of iterations allowed
        params_maxDiffLL = 1E-4;                %Likelihood increase threshold to stop the algorithm
        params_diagRegFact = 1E-4;             %Regularization term is optional
        params_updateComp = ones(3,1);     %pi,Mu,Sigma
        % Protected variable
        queryIndices;                      % Indices of the query data in GMR
        outIndices;                         % Indices of the data out in tangent space in GMR
        outIndicesM;                      % Indices of the data out in manifold in GMR
        nIterGN = 10;                     % Number of iteration for the Gauss Newton algorithm
        nIterEM = 10;                     % Number of iteration for the EM algorithm
    end
    
    methods
        function obj = RiemannianGMMZero(nKernel,nVar)
            %RiemannianGMMZero Assign num. of kernels and dimension of data
            %   nKernel: integer, the num. of Gaussians, K
            %   nVar: integer, the num. / dim. of states
            %   including query variable, D
            obj.nKernel = nKernel;
            obj.nVar = nVar;
            obj.nVarM = obj.nVar+1;
            obj.Mu = zeros(nVar,nKernel);
            obj.MuMan = zeros(obj.nVarM,nKernel);
            obj.Sigma = zeros(nVar,nVar,nKernel);
            obj.Prior = zeros(1,nKernel);
            obj.queryIndices = 1;
            obj.outIndices = (2:obj.nVar)';
            obj.outIndicesM = (2:obj.nVarM);
        end
        
        function obj = initGMMKBins(obj,Data,M,N)
            %initGMMKBins Initialize the GMM before EM by clustering an
            %ordered dataset into equal bins.
            %   Data: D x (N * M), data of all demonstrations (Query must
            %   be included when exits)
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
            obj.MuMan = [obj.Mu(1,:); obj.expmap(obj.Mu(2:end,:), obj.center)]; %Center on the manifold
            obj.Mu = zeros(obj.nVar,obj.nKernel); %Center in the tangent plane at point MuMan of the manifold
        end
        
        function [obj, DataMTmp] = learnGMM(obj, RData)
            %learnGMM Learn the GMM by EM-algorithm xIn xOut x u
            %   RData: RiemannianData struct
            %   data: D x (N * M), data of all demonstrations (Query must
            %   be included when exits)
            %   M: integer, num. of demonstrations
            %   N: integer, num. of data in each demonstration
            %   DataMTmp: D x (N*M) x K, data in the tangent space
            NM = size(RData.data,2);
            DataMTmp = zeros(obj.nVar,NM,obj.nKernel);
            for nb=1:obj.nIterEM
                %E-step
                L = zeros(obj.nKernel,size(RData.dataM,2));
                for i=1:obj.nKernel
                    L(i,:) = obj.Prior(i) * obj.gaussPDF([RData.queryIn-obj.MuMan(1,i); ...
                        obj.logmap(RData.dataM, obj.MuMan(2:end,i))], obj.Mu(:,i), obj.Sigma(:,:,i));
                end
                GAMMA = L ./ repmat(sum(L,1)+realmin, obj.nKernel, 1);
                GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,NM);
                %M-step
                for i=1:obj.nKernel
                    %Update Priors
                    obj.Prior(i) = sum(GAMMA(i,:)) / (NM);
                    %Update MuMan
                    for n=1:obj.nIterGN
                        DataMTmp(:,:,i) = [RData.queryIn-obj.MuMan(1,i); obj.logmap(RData.dataM, obj.MuMan(2:end,i))];
                        obj.MuMan(:,i) = [(obj.MuMan(1,i)+DataMTmp(1,:,i))*GAMMA2(i,:)'; ...
                            obj.expmap(DataMTmp(2:end,:,i)*GAMMA2(i,:)', obj.MuMan(2:end,i))];
                    end
                    %Update Sigma
                    obj.Sigma(:,:,i) = DataMTmp(:,:,i) * diag(GAMMA2(i,:)) * DataMTmp(:,:,i)' + eye(size(obj.Mu,1)) * obj.params_diagRegFact;
                end
            end
        end
        
        function [expDataM, expData, expSigma] = GMR(obj,queryIn)
            %GMR    Gaussian mixture regression
            %   queryIn: Din x N, query variable
            %   expDataM: DM x N, 
            %   expData: Dout x N, 
            %   expSigma: Dout x Dout x N, 

            % Data storation
            in = obj.queryIndices;
            out = obj.outIndices;
            outM = obj.outIndicesM;
            Dout = length(out);
            DoutM = length(outM);
            N = size(queryIn,2);
            expData = zeros(Dout,N);
            expDataOut = zeros(Dout,obj.nKernel,N);
            expDataM = zeros(DoutM,N);
            SigmaTmp = zeros(obj.nVar,obj.nVar,obj.nKernel);
            expSigma = zeros(Dout,Dout,N);
%                         in=1; out=2:4; outMan=2:5;
%             nbVarOut = length(out);
%             nbVarOutMan = length(outMan);            
%             uhat = zeros(nbVarOut,nbData);
%             xhat = zeros(nbVarOutMan,nbData);
%             uOut = zeros(nbVarOut,model.nbStates,nbData);
%             SigmaTmp = zeros(model.nbVar,model.nbVar,model.nbStates);
%             expSigma = zeros(nbVarOut,nbVarOut,nbData);

            %Version with single optimization loop
            for t=1:N
                %Compute activation weight
                for i=1:obj.nKernel
                    H(i,t) = obj.Prior(i) * obj.gaussPDF(queryIn(:,t)-obj.MuMan(in,i), obj.Mu(in,i), obj.Sigma(in,in,i));
                end
                H(:,t) = H(:,t) / sum(H(:,t)+realmin);
                
                %Compute conditional mean (with covariance transportation)
                if t==1
                    [~,id] = max(H(:,t));
                    expDataM(:,t) = obj.MuMan(outM,id); %Initial point
                else
                    expDataM(:,t) = expDataM(:,t-1);
                end
                for n=1:obj.nIterGN
                    for i=1:obj.nKernel
                        %Transportation of covariance from obj.MuMan(outM,i) to expDataM(:,t)
                        Ac = obj.transp(obj.MuMan(outM,i), expDataM(:,t));
                        SigmaTmp(:,:,i) = blkdiag(1,Ac) * obj.Sigma(:,:,i) * blkdiag(1,Ac)';
                        %Gaussian conditioning on the tangent space
                        expDataOut(:,i,t) = obj.logmap(obj.MuMan(outM,i), ...
                            expDataM(:,t)) + SigmaTmp(out,in,i)/SigmaTmp(in,in,i) * (queryIn(:,t)-obj.MuMan(in,i));
                    end
                    expData(:,t) = expDataOut(:,:,t) * H(:,t);
                    expDataM(:,t) = obj.expmap(expData(:,t), expDataM(:,t));
                end
          
                % 	%Compute conditional covariances (by ignoring influence of centers uOut(:,i,t))
                % 	for i=1:obj.nKernel
                % 		expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaTmp(out,out,i) - SigmaTmp(out,in,i)/SigmaTmp(in,in,i) * SigmaTmp(in,out,i));
                % 	end
                
                %Compute conditional covariances (note that since uhat=0, the final part in the GMR computation is dropped)
                for i=1:obj.nKernel
                    SigmaOutTmp = SigmaTmp(out,out,i) - SigmaTmp(out,in,i)/SigmaTmp(in,in,i) * SigmaTmp(in,out,i);
                    expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaOutTmp + expDataOut(:,i,t) * expDataOut(:,i,t)');
                end
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary functions
        function [obj] = setQueryIndices(obj,queryIndices)
            %setQueryIndices Set the query indices for GMR
            %   queryIndices: Din x 1, query indices in column vector
            obj.queryIndices = round(queryIndices);
            Din = length(obj.queryIndices);
            obj.outIndices = (Din+1 : obj.nVar)';
            obj.outIndicesM = (Din+1 : obj.nVarM)';
        end
        function [obj] = setMaxIter(obj, nIterEM, nIterNG)
            %setMaxIter Set the maximum num. of iteration. Assign [] to any
            %arguments to ignore its setting
            %   nIterEM: integer > 1, maximum num. of EM iteration
            %   nIterNG: integer > 1, maximum num. of NG iteration
            if ~isempty(nIterEM)
                obj.nIterEM = ceil(nIterEM);
            end
            if ~isempty(nIterNG)
                obj.nIterGN = ceil(nIterNG);
            end
        end
        function RiemannianData = constructRiemannianData(obj,dataM,data,queryIn)
            %constructRiemannianData Construct the RiemannianData struct.
            %You DO NOT need to assign each arguments. Those related to GMR
            %can be ignored by assign [].
            %   dataM: DM x N
            %   data: Dout x N
            %   queryIn: Din x N
            RiemannianData = [];
            RiemannianData.dataM = dataM(1:4,:);
            RiemannianData.data = data(1:3,:);
            RiemannianData.queryIn = queryIn;
            if ~isempty(queryIn)
                % You need GMR?
                RiemannianData.expDataM = [];
                RiemannianData.expData = [];
                RiemannianData.expSigma = [];
            end
        end
        function [quatOut] = quatRegulate(obj,quatIn,mod)
            %quatRegulate Regulate the unit quaternion to avoid double
            %cover problem
            %   quatIn: 4 x N, unit quaternions [w,x,y,z]'
            %   mod: boolean, true for positive real part, false for
            %   negative real part (default:true)
            %   quatOut: 4 x N, unit quaternions [w,x,y,z]'
            quatOut = quatIn;
            if nargin < 3
                mod = true;
            end
            N = size(quatIn,2);
            if mod
                % Positive real part
                for i = 1:N
                    if quatOut(1,i) < 0
                        quatOut(1,i) = -quatOut(1,i);
                    end
                end
            else
                % Negative real part
                for i = 1:N
                    if quatOut(1,i) > 0
                        quatOut(1,i) = -quatOut(1,i);
                    end
                end
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary functions from pbd-lib
        % Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
        % Written by Sylvain Calinon, http://calinon.ch/
        %
        % These functions are part of PbDlib, http://www.idiap.ch/software/pbdlib/
        function Q = QuatMatrix(obj,q)
            %QuatMatrix Generate quaternion matrix for mapping
            Q = [q(1) -q(2) -q(3) -q(4);
                q(2)  q(1) -q(4)  q(3);
                q(3)  q(4)  q(1) -q(2);
                q(4) -q(3)  q(2)  q(1)];
        end
        
        function acosx = acoslog(obj,x)
            %acoslog Arcosine re-defitinion to make sure the distance
            %between antipodal quaternions is zero (2.50 from Dubbelman's Thesis) 
            for n=1:size(x,2)
                % sometimes abs(x) is not exactly 1.0
                if(x(n)>=1.0)
                    x(n) = 1.0;
                end
                if(x(n)<=-1.0)
                    x(n) = -1.0;
                end
                if(x(n)>=-1.0 && x(n)<0)
                    acosx(n) = acos(x(n))-pi;
                else
                    acosx(n) = acos(x(n));
                end
            end
        end
        
        function Log = logfct(obj,x)
            %logfct
            % 	scale = acos(x(3,:)) ./ sqrt(1-x(3,:).^2);
            scale = obj.acoslog(x(1,:)) ./ sqrt(1-x(1,:).^2);
            scale(isnan(scale)) = 1;
            Log = [x(2,:).*scale; x(3,:).*scale; x(4,:).*scale];
        end
        
        function Exp = expfct(obj,u)
            %expfct
            normv = sqrt(u(1,:).^2+u(2,:).^2+u(3,:).^2);
            Exp = real([cos(normv) ; u(1,:).*sin(normv)./normv ; u(2,:).*sin(normv)./normv ; u(3,:).*sin(normv)./normv]);
            Exp(:,normv < 1e-16) = repmat([1;0;0;0],1,sum(normv < 1e-16));
        end
        
        function u = logmap(obj, x, mu)
            %logmap
            if norm(mu-[1;0;0;0])<1e-6
                Q = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            else
                Q = obj.QuatMatrix(mu);
            end
            u = obj.logfct(Q'*x);
        end
        
        function x = expmap(obj, u, mu)
            %expmap
            x = obj.QuatMatrix(mu) * obj.expfct(u);
        end
        
        function Ac = transp(obj, g,h)
            %transp
            E = [zeros(1,3); eye(3)];
            vm = obj.QuatMatrix(g) * [0; obj.logmap(h,g)];
            mn = norm(vm);
            if mn < 1e-10
                disp('Angle of rotation too small (<1e-10)');
                Ac = eye(3);
                return;
            end
            uv = vm / mn;
            Rpar = eye(4) - sin(mn)*(g*uv') - (1-cos(mn))*(uv*uv');
            Ac = E' * obj.QuatMatrix(h)' * Rpar * obj.QuatMatrix(g) * E; %Transportation operator from g to h
        end
        
        function prob = gaussPDF(obj, Data, Mu, Sigma)
            %gaussPDF Likelihood of datapoint(s) to be generated by a Gaussian parameterized by center and covariance.
            %	Data:  D x N array representing N datapoints of D dimensions.
            %	Mu:    D x 1 vector representing the center of the Gaussian.
            %	Sigma: D x D array representing the covariance matrix of the Gaussian.
            %	prob:  1 x N vector representing the likelihood of the N datapoints.
            [D,N] = size(Data);
            Data = Data - repmat(Mu,1,N);
            prob = sum((Sigma\Data).*Data,1);
            prob = exp(-0.5*prob) / sqrt((2*pi)^D * abs(det(Sigma)) + realmin);
        end
        
    end
end

