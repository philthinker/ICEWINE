classdef SEDSZero < GMMOne
    %SEDSZero Stable estimator of dynamic systems
    %   Maker sure that the path 'GaussianMixtureModel' has been added.
    %
    %   Haopeng Hu
    %   2019.12.25 Merry Christmas!
    %   All rights reserved
    %
    %   Notations:
    %   |   D: dimension of state
    %   |   Din: dimension of query state
    %   |   Dout: dimension of output state, Dout = D - Din
    %   |       Note that in most cases Din == Dout.
    %   |   N: num. of data
    %   |   K: num. of kernels
    
    properties (Access = public)
        dt;         % scalar, time step
        A;          % D x D x K, A matrices
        b;          % D x K, b vectors
    end
    
    properties (Access = protected)
        
    end
    
    methods
        function obj = SEDSZero(nVar,nKernel,dt)
            %SEDSZero Init. the SEDS with D, K and dt. Never forget that
            %the time series must be in the first row.
            %   nVar:integer, D
            %   nKernel: integer, K
            %   dt: scalar, time step (default: 1e-3)
            obj = obj@GMMOne(floor(nKernel),floor(nVar));
            if nargin < 3
                dt = 1e-3;
            end
            obj.dt = dt;
            obj.A = [];
            obj.b = [];
            obj = obj.setGMRInOut((1:floor(nVar/2))');  % We assume that positions take the first D/2 rows
        end
        
        function obj = preOptim(obj,Data,M,N)
            %preOptim Compute the initial guess of the parameters
            %   Data: D x (M * N), demo data (centered at zero)
            %   M: integer, M
            %   N: integer, N
            obj = obj.initGMMKBins(Data,M,N);
            obj = obj.learnGMM(Data);
        end
        
        function obj = Refine(obj,Data)
            %Optim Constrained optimization
            %   Data: D x (M * N), demo data (centered at zero)
            disp('Refinement through constrained optimization...');
            in = obj.Datain;
            out = obj.Dataout;
            % Compute H
            H = zeros(size(Data,2),obj.nKernel);
            for i=1:obj.nKernel
                H(:,i) = obj.Prior(i) * obj.GaussPDF(Data(in,:),obj.Mu(in,i),obj.Sigma(in,in,i));
            end
            H = H ./ repmat(sum(H,2),1,obj.nKernel);
            % Optimization
            options = optimset('Algorithm','active-set','display','notify'); %'large-scale' could be used instead 'active-set'
            X=Data(in,:); Y=Data(out,:); 
            obj.A = zeros(length(out),length(in),obj.nKernel);
            obj.b = zeros(length(out),obj.nKernel);
            for i=1:obj.nKernel
                %Initialization through weighted least-squares regression (dx=A*x)
                A0 = [(X * diag(H(:,i).^2) * X') \ X * diag(H(:,i).^2) * Y']';
                %Refined solution through constrained optimization
                obj.A(:,:,i) = fmincon(@(A)myfun(A,X,Y,diag(H(:,i))),A0,[],[],[],[],[],[],@(A)mycon(A),options);
                obj.b(:,i) = -obj.A(:,:,i)*Data(in,end);    % In most cases, we can set it to be zero as the convergence point is always the origin.
            end
            
            % Objective func.
            function f = myfun(A,X,Y,W)
                fTmp = (A*X-Y)*W;
                f = norm(reshape(fTmp,size(Y,1)*size(Y,2),1));
            end
            % Constraint func.
            function [c,ceq] = mycon(A)
                Atmp = (A+A')*.5;
                [~,D] = eig(Atmp);
                for j=1:size(A,1)
                    polesR(j) = real(D(j,j)) + 1E-1; %margin on "negativity"
                end
                %Force the poles to be in the left plane
                c = [polesR];
                ceq = 0;
            end
        end
        
        function [expData,expDx] = DP(obj,Data0,N)
            %DP Generate the position and velocity states by dynamic
            %programming.
            %   Data0: Din x 1, init. state
            %   N: integer, num. of states to be generated
            %   expData: Din x N, expectation data (position)
            %   expDx: Dout x N, expectation data (velocity)
            expData = repmat(Data0,[1,N]);
            expDx = zeros(length(obj.Dataout),N);
            h = zeros(1,obj.nKernel);
            in = obj.Datain;
            % Dynamic programming
            for t = 2:N
                % Compute weighting h
                for i = 1:obj.nKernel
                    h(i) = obj.Prior(i) * obj.GaussPDF(expData(:,t-1),obj.Mu(in,i),obj.Sigma(in,in,i));
                end
                h = h/sum(h);
                % Compute velocity
                for i = 1:obj.nKernel
                    expDx(:,t) = expDx(:,t) + h(i) * (obj.A(:,:,i) * expData(:,t-1) + obj.b(:,i));
                end
                % Compute position
                expData(:,t) = expData(:,t-1) + obj.dt * expDx(:,t);
            end
        end
        
        function [expData,expSigma,expDx] = GMR(obj,Data0,N)
            %GMR Standard Gaussian mixture regression
            %   Data0: Din x 1, the init. state
            %   N: integer, num. of states to be queried
            %   expData: Din x N, expectation data (position)
            %   expSigma: Din x Dout x N, covariances
            %   expDx: Din x N, expectation data (velocity)
            in = length(obj.Datain);
            out = length(obj.Dataout);
            expData = zeros(in,N); expSigma = zeros(in,in,N); expDx = zeros(out,N);
            expData(:,1) = Data0;
            for i = 2:N
                [expDx(:,i),expSigma(:,:,i)] = GMR@GMMOne(obj,expData(:,i-1));
                expData(:,i) = expData(:,i-1) + obj.dt * expDx(:,i);
            end
        end
        
    end
end

