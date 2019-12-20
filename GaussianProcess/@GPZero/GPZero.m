classdef GPZero
    %GPZero Gaussian Process
    %   Haopeng Hu
    %   2019.12.19
    %   All rights reserved
    %   Notations
    %   |   D: dimension of variable
    %   |   N: num. of data
    
    properties (Access = public)
        nVar;   % D
        sigma;  % signal variance
        W;      % covariance matrix of Gaussian kernel
        reg;    % matrix regulator
    end
    
    properties (Access = protected)
        indexGPRIn;     % Integer, index of query variable in data
        indexGPROut;    % Integers, indices of regression variable in data
        Data;           % Demonstration data
    end
    
    methods
        function obj = GPZero(Data)
            %GPZero Init. Gaussian process with D
            %   Data: D x N, the demonstration data. We always assume that
            %   the first row is the array of query variable.
            obj.nVar = size(Data,1);
            obj.indexGPRIn = 1;
            obj.indexGPROut = (2:obj.nVar);
            obj.Data = Data;
        end
        
        function [expData,expSigma] = GPR(obj,query)
            %GPR Gaussian process regression
            %   query:1 x N, query points
            outputArg = obj.Property1 + query;
        end
        
        function obj = setParam(obj,sigma,W,regulator)
            %setParam Set the parameters of the GP
            %   sigma: scalar: signal variance
            %   W: covariance matrix of Gaussian kernel
            %   regulator: scalar, matrix regulator parameter (optional)
            obj.sigma = sigma;
            obj.W = W;
            if nargin >= 4
                obj.reg = regulator;
            end
        end
    end
end

