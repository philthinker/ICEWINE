classdef DMPdbGMM < GMMOne
    %DMPdbGMM Dynamic movement primitive which is driven by Gaussian
    %mixture model based forcing term.
    %   alphax, scalar
    %   alpha
    %   beta
    %   dt
    %   K
    
    %   addpath('GaussianMixtureModel');
    
    properties
        Property1
    end
    
    methods
        function obj = DMPdbGMM(alphax,alpha,beta,dt,K)
            %UNTITLED 构造此类的实例
            %   此处显示详细说明
            obj.Property1 = alphax + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

