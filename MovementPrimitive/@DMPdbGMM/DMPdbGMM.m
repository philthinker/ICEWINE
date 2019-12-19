classdef DMPdbGMM < GMMOne
    %DMPdbGMM Dynamic movement primitive which is driven by Gaussian
    %mixture model based forcing term.
    %   alphax, positive scalar, decaying coefficient of canonical system
    %   alpha,  positive scalar, decaying coefficient
    %   beta,   positive scalar, decaying coefficient
    %   dt,     positive scalar, time difference
    %   K,      integer, num. of Gaussian kernels
    %
    %   addpath('GaussianMixtureModel');
    %
    %   Here we use Ijspeert's DMP as driven system. That is
    %   tau \dot{x} = -alphax x
    %   tau \dot{y} = z
    %   tau \dot{z} = alpha ( beta ( g - y ) - z ) + f(x)
    %   f(x) = x(g-y(0)) PSI(x)
    
    %   Haopeng Hu
    %   2019.12.08
    %   All rights reserved
    %   haopeng_hu@sina.com
    
    properties (Access = public)
        alphax;
        alpha;
        beta;
        dt;
    end
    
    properties (Access = protected)
        tau;
        y0;
        g;
    end
    
    methods
        function obj = DMPdbGMM(alphax,alpha,beta,dt,K)
            %DMPdbGMM Init. DMPdbGMM
            %   alphax, positive scalar, decaying coefficient of canonical system
            %   alpha,  positive scalar, decaying coefficient
            %   beta,   positive scalar, decaying coefficient
            %   dt,     positive scalar, time difference
            %   K,      integer, num. of Gaussian kernels
            obj = obj@GMMOne(K,2);
            obj.alphax = alphax;
            obj.alpha = alpha;
            obj.beta = beta;
            obj.dt = dt;
            obj.tau = 1;
            obj.y0 = 0;
            obj.g = 0;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

