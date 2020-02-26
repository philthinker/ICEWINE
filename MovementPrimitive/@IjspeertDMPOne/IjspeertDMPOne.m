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
    %
    %   For more details, refer to
    %   'https://blog.csdn.net/philthinker/article/details/88366726'
    
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
        
        function [Y,x,fx] = run(obj,y0,g,N)
            %run Run the DMP given the num. of data
            %   y0: scalar, initial position
            %   g: scalar, goal position
            %   N: integer, num. of data
            %   Y: N x 3, [y dy ddy]
            %   x: N x 1, x
            %   fx: N x 1, forcing term
            N = round(N);
            x = obj.genX(N);
            [Y,fx] = obj.genY(x,y0,g);
        end
        
        function [x,ddy,dy,y] = runItera(obj,x,y,dy,y0,g,dt)
            %runItera Run the DMP iteratively
            %   x: scalar, current driving signal
            %   y: scalar, current position
            %   y0: scalar, initial position
            %   g: scalar, goal position
            %   dt: scalar, time difference
            % Hyper param
            TAU = obj.tau;
            ALPHA = obj.alpha;
            BETA = obj.beta;
            ALPHAX = obj.alphax;
            % Update ddy
            fx = obj.forcingFunc(x,y0,g);
            ddy = (1/(TAU^2))*(ALPHA*(BETA*(g-y)-TAU*dy)+fx);
            % Update dy, y and x
            dy = dy + dt * ddy;
            y = y + dt * dy;
            dx = -(1/TAU) * ALPHAX * x;
            x = x + dt * dx;
        end
        
        function obj = learnLWR(obj,Demo)
            %learnLWR Learn the DMP by LWR.
            %   Demo: N x 3, the demo [y dy ddy]
            % Hyper param.
            TAU = obj.tau;  % recommended: tau = 1 for DMP learning
            ALPHA = obj.alpha;
            BETA = obj.beta;
            K = obj.nKernel;
            % Demo data
            yd = Demo(:,1); dyd = Demo(:,2); ddyd = Demo(:,3);
            y0d = yd(1); gd = yd(end);
            N = length(yd);
            % Compute demo forcing term
            fd = (TAU^2) .* ddyd - ALPHA * (BETA * (gd - yd) - TAU .* dyd);
            % Compute basis
            x = obj.genX(N);
            xi = x .* (gd - y0d);
            Psi = zeros(N,K);
            for i = 1:K
                Psi(:,i) = obj.basisFunc(x,obj.c(i),obj.h(i));
            end
            % Learn weights by minimizing weighted quadratic cost
            for i = 1:K
                T = diag(Psi(:,i));
                obj.w(i) = (xi' * T * xi)\(xi' * T * fd);
            end
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
        function [] = plotDMP(obj,x,Y)
            %plotDMP Plot the output of the DMP
            figure;
            t = linspace(0,1,length(x));
            % plot x
            subplot(2,2,1);
            plot(t,x); title('x');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
            % plot y
            subplot(2,2,2);
            plot(t,Y(:,1)); title('y');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
            % plot dy
            subplot(2,2,3);
            plot(t,Y(:,2)); title('dy');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
            % plot ddy
            subplot(2,2,4);
            plot(t,Y(:,3)); title('ddy');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
        end
        function [] = plotComparison(obj,Y,Demo)
            %plotComparison Plot the learned Y and its demo together
            %   Y: N x 3, Y generated by obj.genY()
            %   Demo: N x 3, the demo
            t = linspace(0,1,size(Y,1))';
            figure;
            % plot y and yd
            subplot(3,1,1);
            plot(t,[Demo(:,1),Y(:,1)]); legend('yd','y');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
            % plot dy and dyd
            subplot(3,1,2);
            plot(t,[Demo(:,2),Y(:,2)]); legend('dyd','dy');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
            % plot ddy and ddyd
            subplot(3,1,3);
            plot(t,[Demo(:,3),Y(:,3)]); legend('ddyd','ddy');
            aa = axis; axis([min(t),max(t),aa(3:4)]);
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
        [Y,fx] = genY(obj,x,y0,g);         % Dynamic system
    end
end

