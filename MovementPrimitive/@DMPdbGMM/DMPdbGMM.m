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
            %UNTITLED ��������ʵ��
            %   �˴���ʾ��ϸ˵��
            obj.Property1 = alphax + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 �˴���ʾ�йش˷�����ժҪ
            %   �˴���ʾ��ϸ˵��
            outputArg = obj.Property1 + inputArg;
        end
    end
end

