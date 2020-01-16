classdef ProMPZero
    %ProMPZero Probabilistic Movement Primitive
    %   - Hierarchical Bayesian Model
    %   - Trajectory Distributions Learned from Stochastic Movements
    %   - We assume that the drive signal z is a linear variable
    %   For more details, refer to A. Paraschos "Using Probabilistic
    %   Movement Primitives in Robotics". Autonomous Robots, 2018.
    %
    %   We assume that all probabilistic distributions are Gaussian.
    %   It is designed for stroke-based movement ONLY.
    %   We ONLY consider 1-D data for each DoF. 2-D data, such as position
    %   concatened with velocity, are not supported.
    %
    %   Haopeng Hu
    %   2020.01.01 Happy New Year!
    %   All rights reserved
    %
    %   Notations:
    %   |   N:  Num. of data
    %   |   D:  Dim. of data or DoF
    %   |   M:  Num. of demos
    %   |   K:  Num. of kernels
    
    properties (Access = public)
        % Model parameters
        Muw;        % Mean value of weight vector w
        Sigmaw;     % Variance of weight vector w
        Sigmay;     % Variance of zero-mean noise
        % Variables
        c;          % Centers of basis functions
        h;          % Variances of basis functions
    end
    
    properties (Access = protected)
        nVar;
        nKernel;
        params_diagRegFact = 1E-9;  %Regularization term is optional
    end
    
    methods (Access = public)
        % Initialization, Learning and Reproduction
        function obj = ProMPZero(nVar,nKernel,h)
            %ProMPZero Init. the ProMP with D
            %   D: integer, D
            %   K: integer, K
            %   h: positive scalar, the variance (optional)
            if nargin < 3
                h = 0.001;
            end
            obj.nVar = ceil(nVar);
            obj.nKernel = ceil(nKernel);
            obj.Muw = zeros(obj.nKernel * obj.nVar,1);
            obj.Sigmaw = zeros(obj.nKernel * obj.nVar,obj.nKernel * obj.nVar);
            obj.Sigmay = zeros(obj.nVar);
            h = abs(h);
            obj.h = ones(1,obj.nKernel)*h;
            obj.c = linspace(-2*h,1+2*h,obj.nKernel);
        end
        
        function obj = leanLRR(obj,Demos)
            %leanLRR Learn the param. by linear ridge regression and maximum
            %likelihood estimation.
            %   Demos: 1 x M struct array, where
            %   |   Demos.data: D x N, demo data
            M = length(Demos);
            D = obj.nVar;
            K = obj.nKernel;
            w = zeros(D * K,M);
            % Locally weighted regression
            for i = 1:M
                N = size(Demos(i).data,2);
                z = linspace(0,1,N);
                Phi = obj.genBasis(z);
                for j = 1:D
                    w((j-1)*K+1:j*K,i) = (Phi'*Phi + obj.params_diagRegFact*eye(K))\Phi'*(Demos(i).data(j,:))';
                end
            end
            % Maximum likelihood
            Mu = mean(w,2);
            Sigma = zeros(K*D, K*D);
            for i = 1:M
                Sigma = Sigma + (w(:,i)-Mu)*((w(:,i)-Mu)');
            end
            Sigma = Sigma/M;
            obj.Muw = Mu;
            % Compute lambdaw s.t. Sigmaw is PD
            lambdaw = 0;
            d = eig(M*Sigma);
            while ~(all(d>0))
                lambdaw = lambdaw + 1e-3;
                d = eig(M*Sigma + lambdaw*eye(D*K));
            end
            obj.Sigmaw = (M*Sigma+lambdaw*eye(D*K))./(M+lambdaw);
        end
        
        function [expData,expSigma] = reproduct(obj,N)
            %reproduct Reproduct what it has learned
            %   N: integer, the num. of data to be reproducted
            %   expData: D x N, reproducted expectation data
            %   expSigma:D x D x N, reproducted variances
            z = linspace(0,1,N);
            D = obj.nVar;
            K = obj.nKernel;
            expData = zeros(D,N);
            expSigma = zeros(D,D,N);
            Phi = obj.genBasis(z);
            for d = 1:D
                expData(d,:) = (Phi*obj.Muw((d-1)*K+1:d*K))';
            end
            for n = 1:N
                % Compute Psi. Note that obly 1-D data is supported.
                Psi = kron(eye(D),Phi(n,:));
                expSigma(:,:,n) = Psi*(obj.Sigmaw)*(Psi')+obj.Sigmay;
            end
        end
    end
    
    methods (Access = public)
        % Modulation, Co-Activation and Blending/Sequencing
        % Temporal modulation is achieved by phase variable
        function [obj] = modulate(obj,viaPoints)
            %modulate The modulation operation of ProMP
            %   viaPoints: 1 x Nv struct arrary, the via-points in which:
            %   |   viaPoints.data: D x Nv, the via-points, y*
            %   |   viaPoints.t: 1 x Nv, the temporal indices (phase variable 0 to 1)
            %   |   viaPoints.Sigma: D x D x Nv, the variances of via-points (default:zeros(D,D,Nv))
            %   Note that if you DO NOT wanna change the learned model,
            %   assign a new model to the output argument obj.
            %   Theoretically, you can condition on any subset of the
            %   state, but it is not supported by this method.
            Nv = length(viaPoints);
            if ~isfield(viaPoints,'Sigma')
                viaPoints.Sigma = zeros(obj.nVar,obj.nVar,Nv);
            end
            for i = 1:Nv
                % Compute Phi
                Phi_t = obj.genBasis(viaPoints(i).t);
                % Compute Psi
                Psi = kron(eye(obj.nVar),Phi_t);
                % Bayes theorem ???
                L = obj.Sigmaw*Psi'/(viaPoints(i).Sigma + Psi*obj.Sigmaw*Psi');
                Muw1 = obj.Muw + L*(viaPoints.data - Psi*obj.Muw);
                Sigmaw1 = obj.Sigmaw - L*Psi*obj.Sigmaw;
                obj.Muw = Muw1;
                obj.Sigmaw = Sigmaw1;
            end
        end
        function [obj] = coactivate(obj,Models)
        end
        function [obj] = blend(obj,Models,alpha)
        end
    end
        
    methods (Access = public)
        % Auxiliary Functions
        function obj = setNoise(obj,epsilony)
            %setNoise Set the variance of the zero-mean noise
            %   epsilony: scalar, the magnitude of noise
            epsilony = abs(epsilony);
            obj.Sigmay = eye(obj.nKernel * obj.nVar)*epsilony;
        end
        function obj = setBasis(obj,c,h)
            %setBasis Set the centers, variance and decaying coefficient
            %   c: 1 x K, centers from 0 to 1
            %   h: 1 x K positive scalars, inverse of variances
            obj.c = abs(c);
            obj.h = abs(h);
        end
        % Figure
        function h = plotBasis(obj)
            %plotBasis Plot the basis func.
            z = linspace(0,1,200);
            Phi = obj.genBasis(z);
            for i = 1:obj.nKernel
                h = plot(z,Phi(:,i));
                hold on;
            end
            grid on;
        end
    end
    
    methods (Access = protected)
        [Phi] = genBasis(obj,z);
    end
end

