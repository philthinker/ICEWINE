classdef IjspeertDMPOne
    %IjspeertDMPOne Ijspeert's dynamic movement primitive
    %   DMP designed for discrete (one-stroke) movements only.
    %   tau dx = -alphax x
    %   tau dy = z
    %   tau dz = alpha(beta(g-y)-z)+f(x)
    %   Notations:
    %   |   K: num. of basis func.
    %   |   N: num. of data
    %   
    %   Haopeng Hu
    %   2020.02.25
    %   All rights reserved
    
    properties (Access = public)
        w;      % weights
        alphax; % alphax
        alpha;  % alpha
        beta;   % beta
        tau;    % temporal rescaling factor
    end
    
    properties (Access = protected)
        c;      % centers of basis
        h;      % variances of basis
        nKernel;% num. of basis functions
        mode;   % driving mode
    end
    
    methods
        function obj = IjspeertDMPOne(nKernel,mode)
            %IjspeertDMP2 Init. the dmp with num. of basis and driving
            %mode. Note here all the data are vectors.
            %   nKernel: integer, num. of basis
            %   mode: integer, 
            %   |   0: dynamic canonical system (default)
            %   |   1: linear x sequence
            if nargin < 2
                mode = 0;
            end
            obj.nKernel = round(nKernel);
            obj.w = zeros(obj.nKernel,1);
            obj.tau = 1;
            obj.alphax = 8; % default param
            obj.alpha = 25; % default param
            obj.beta = 5;   % default param
            % c, h assignment. Note that we always assume that x: 1 to 0
            obj.mode = mode;
            if mode == 1
                % x: Linear sequence
                h = 0.001;  % default param
                obj.h = h * ones(obj.nKernel,1);
                obj.c = linspace(-2*h,1+2*h,obj.nKernel);
            else
                % x: Dynamic system
                h = 0.4;  % default param
                [obj.c, obj.h] = obj.centersAssign(h);
            end
        end
        
        function outputArg = run(obj,y0,g,N)
            %run Run the DMP
            %   此处显示详细说明
            outputArg = obj.Property1 + N;
        end
        
        function [y,dy,ddy] = runItera(obj,y,dy,y0,g,dt)
            %runItera Run the DMP iteratively
        end
        
        function outputArg = learnLWR(obj,Demo)
            %learnLWR Learn the DMP by LWR
            %   此处显示详细说明
            outputArg = obj.Property1 + Demo;
        end
    end
    
    methods (Access = public)
        % Figure
        function [] = plotBasis(obj,fx)
            %plotBasis Plot the basis function and forcing term
            %   fx: N x 1, values of the forcing term (optional)
            N = 1000;
            K = obj.nKernel;
            t = linspace(0,1,N);
            x = obj.genX(N);
            Psi = zeros(N,K);
            for i = 1:K
                Psi(:,i) = obj.basisFunc(x,obj.c(i),obj.h(i));
            end
            if nargin < 2
                % Only plot the basis func.
                figure;
                plot(t,Psi); aa = axis;
                axis([min(t),max(t),aa(3:4)]);
            else
                % Plot basis func. and fx
                figure;
                subplot(2,1,1);
                plot(t,Psi); aa = axis;
                axis([min(t),max(t),aa(3:4)]);
                subplot(2,1,2);
                plot(t,fx); aa = axis;
                axis([min(t),max(t),aa(3:4)]);
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary functions
        function obj = setCanonicalParam(obj,alphax)
            %setCanonicalParam Set alphax of the canonical system.
            %   alphax: scalar, canonical param.
            obj.alphax = abs(alphax);
        end
        function obj = setDynamicParam(obj,alpha,beta)
            %setDynamicParam Set alpha and beta of the dynamic system. We
            %recommended that alpha > 4*beta > 0.
            %   alpha: scalar, alpha
            %   beta: scalar, beta
            obj.alpha = abs(alpha);
            obj.beta = abs(beta);
        end
        function obj = setTemporalFactor(obj,tau)
            %setTemporalFactor Set the tau
            % tau: scalar, temporal scaling factor.
            obj.tau = abs(tau);
        end
        function obj = setVariance(obj,h)
            %setVariance Set the variances h. Note that c will also be
            %re-assigned simultaneously.
            %   h: scalar, variances
            h = abs(h);
            if obj.mode == 1
                % x: linear sequence
                obj.h = h * ones(obj.nKernel,1);
                obj.c = linspace(-2*h,1+2*h,obj.nKernel);
            else
                % x: dynamic system
                [obj.c, obj.h] = obj.centersAssign(h);
            end
        end
    end
    
    methods (Access = protected)
        [centers,variances] = centersAssign(obj,h);
        [x] = genX(obj,N);              % Canonical system
        [psi] = basisFunc(obj,x,c,h);   % Basis func.
        [fx] = forcingFunc(obj,x,y0,g); % Nonlinear forcing term
        [Y] = genY(obj,N);              % Dynamic system
    end
end

