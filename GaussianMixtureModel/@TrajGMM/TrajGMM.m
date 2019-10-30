classdef TrajGMM < GMMOne
    %TrajGMM Gaussian Mixture Model with dynamic features
    %   It can also be extended to its task-parameterized version.
    %   Notations:
    %       D: nVarPos, dimension of demo data
    %       DD: nVar, dimension of dynamic data
    %       K: nKernel, num. of Gaussians
    %       N: nData, num. of data in each demo
    %       M: nSample, num. of demos
    
    %   Haopeng Hu
    %   2019.10.28
    %   All rights reserved
    
    properties
        nDeriv;     % Num. of dynamic features (2 or 3). 
        nVarPos;    % Dimension of the position data
        nSample;    % Num. of demos available
        nData;      % Num. of data in one trajectory
        dt;         % Time step
    end
    
    properties (Access = protected)
        Phi;        % The large sparse matrix \Phi
        Phi1;       % The large sparse matrix \Phi1
    end
    
    methods
        function obj = TrajGMM(nKernel,nVarPos,nDeriv,dt,nSample,nData)
            %TrajGMM Assign the num. of Gaussian kernels and dimension
            %   nKernel: integer, num. of Gaussian kernels
            %   nVarPos: integer, dimension of the position variable
            %   nDeriv: 2 or 3, num. of dynamic features (2 for [x dx]', 3
            %   for [x dx ddx]')
            %   dt: float, time step
            nVar = nVarPos * nDeriv;
            obj = obj@GMMOne(nKernel,nVar);
            obj.dt = dt;
            obj.nDeriv = nDeriv;
            obj.nVarPos = nVarPos;
            obj.nSample = nSample;
            obj.nData = nData;
            [obj.Phi,obj.Phi1] = obj.constructPhi();
        end
       
        function [Data,x,zeta] = dynamicDataGeneration(obj,Demos)
            %dynamicDataGeneration Generate the data and its derivatives
            %   Demos: 1 x M cell, the demos
            %   Data: DD x (M * N), the arranged data with its derivatives
            %   zeta: (N*D) x 1, the vector contains data in column
            %   x: (N*DD) x 1, the vector contains data and its
            %   deriveatives in column
            % Retrieve the raw position data
            Data = obj.dataRegulate(Demos);
            % Re-arrange data in vector form
            x = reshape(Data, (obj.nVarPos)*(obj.nData)*(obj.nSample), 1) * 1E2;            % Scale data to avoid numerical computation problem
            zeta = (obj.Phi) * x;   % y is for example [x1(1), x2(1), x1d(1), x2d(1), x1(2), x2(2), x1d(2), x2d(2), ...]
            Data = reshape(zeta, (obj.nVarPos)*(obj.nDeriv), (obj.nData)*(obj.nSample));    % Include derivatives in Data
        end
        
        function [expData,expSigma] = reproductTrajGMM(obj,query,LSMode)
            %reproductTrajGMM Reproduct the trajectory with trajectory-GMM
            %   query: 1 x N, the query state sequence
            %   LSMode: integer, the computation method of LS
            %   |   0: Using MATLAB func. lscov (Default)
            %   |   1: Most readable but not optimized method
            %   |   2: Using Cholesky and QR decompositions
            
            if nargin < 3
                LSMode = 0;
            end
            
            %Create single Gaussian N(MuQ,SigmaQ) based on query sequence q
            MuQ = reshape(obj.Mu(:,query), (obj.nVar)*(obj.nData), 1);
            SigmaQ = zeros((obj.nVar)*(obj.nData));
            for t=1:obj.nData
                id = ((t-1)*(obj.nVar)+1:t*(obj.nVar));
                SigmaQ(id,id) = obj.Sigma(:,:,query(t));
            end
            if LSMode == 1
                % Most readable but not optimized method
                PHIinvSigmaQ = obj.Phi1' / SigmaQ;
                Rq = PHIinvSigmaQ * (obj.Phi1);
                rq = PHIinvSigmaQ * MuQ;
                zeta = Rq \ rq;             % Can also be computed with c = lscov(Rq, rq)
                expData = reshape(zeta, model.nbVarPos, nbData);  % Reshape data for plotting
                %Covariance Matrix computation of ordinary least squares estimate
                mse =  (MuQ'*SigmaQ\MuQ - rq'*Rq\rq) ./ ((obj.nVar-obj.nVarPos)*(obj.nData));
                S = Rq\mse;
            elseif LSMode == 2
                % Using Cholesky and QR decompositions
                T = chol(SigmaQ)'; %SigmaQ=T*T'
                TA = T \ PHI1;
                TMuQ = T \ MuQ;
                [Q, R, perm] = qr(TA,0); %obj.Phi1(:,perm)=Q*R
                z = Q' * TMuQ;
                zeta = zeros(obj.nData * obj.nVarPos,1);
                zeta(perm,:) = R \ z; %zeta=(TA'*TA)\(TA'*TMuQ)
                expData = reshape(zeta, obj.nVarPos, obj.nbData); %Reshape data for plotting
                %Covariance Matrix computation of ordinary least squares estimate
                err = TMuQ - Q*z;
                mse = err'*err ./ ((obj.nVar) * (obj.nData) - (obj.nVarPos) * (obj.nData));
                Rinv = R \ eye((obj.nVarPos) * (obj.nData));
                S(perm,perm) = Rinv*Rinv' .* mse;
            else
                % Using MATLAB func. lscov (Default)
                [zeta,~,mse,S] = lscov(obj.Phi1, MuQ, SigmaQ, 'chol'); %Retrieval of data with weighted least squares solution
                expData = reshape(zeta, obj.nVarPos, obj.nData); %Reshape data for plotting
            end
            % Rebuild covariance by reshaping S
            expSigma = zeros(obj.nVarPos,obj.nVarPos,obj.nData);
            for t=1:obj.nData
                id = ((t-1) * (obj.nVarPos)+1 : t * (obj.nVarPos));
                expSigma(:,:,t) = S(id,id) * obj.nData;
            end
        end
    end
    
    methods (Access = public)
        function [Data] = dataRegulate(obj,Demos)
            %dataRegulate Regulate the demos data into one matrix
            %   Demos: 1 x M cell, the demos
            %   Data: D x N, the regulated data
            TmpN = zeros(obj.nSample,1);
            for i = 1:obj.nSample
                TmpN(i) = size(Demos{i},2);
            end
            Data = zeros(size(Demos{1},1),sum(TmpN));
            nTemp = 0;
            for i = 1:obj.nSample
                Data(:,nTemp+1:nTemp+TmpN(i)) = Demos{i};
                nTemp = nTemp + TmpN(i);
            end
        end
        
        function [query] = deriveQuery(obj,GAMMA)
            %deriveQuery Derive the query sequence from GAMMA
            %   GAMMA: D x N, the likelihood in E-Step
            %   query: 1 x N, the qeury sequence
            [~,query] = max(GAMMA,[],1);
        end
    end
    
    methods (Access = protected)
        
        [Phi,Phi1,Phi0] = constructPhi(obj,nData,nSample);
        
        [obj,GAMMA2,LL] = TrajEMGMM(obj,Data);
    end
    
    
end

