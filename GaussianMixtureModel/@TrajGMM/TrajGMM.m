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
        dt;         % Time step
    end
    
    properties (Access = protected)
        nSample;    % Num. of demos available
        nData;      % Num. of data in one trajectory
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
       
        function [Data,zeta,x] = dynamicDataGeneration(obj,Demos)
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
        
        function [obj] = initGMMKMeans(obj,Data)
            %initGMMKMeans Initialize the GMM by K-Means algorithm
            %   Data: DD x N, the demo data
            diagRegularizationFactor = 1E-2; %Optional regularization term
            Data = Data';   % For S. Calinon's habit
            [idList,Mu] = obj.kMeans(Data);  % K-Means clustering
            obj.Mu = Mu';
            for i = 1:obj.nKernel
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                obj.Sigma(:,:,i) = cov([Data(idtmp,:);Data(idtmp,:)]);
                % Optional regularization term to avoid numerical
                % instability
                obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(obj.nVar)*diagRegularizationFactor;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
        end
        
        function [obj] = learnGMM(obj,Data)
            %learnGMM Learn the GMM by EM algorithm
            %   Data: DD x N, the demo data
            obj = obj.EMGMMZero(Data);
        end
        
    end
    
    methods (Access = protected)
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
        
        [Phi,Phi1,Phi0] = constructPhi(obj,nData,nSample);
        
        [obj,GAMMA2,LL] = TrajEMGMM(obj,Data);
    end
    
    
end

