classdef GPZero
    %GPZero Gaussian Process
    %   We assume that the data are column vectors with the first element
    %   being the temporal item.
    %   Standard process: GPZero -> setParam -> preGPR -> GPR
    %
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
        invK;   % inverse of K matrix
    end
    
    properties (Access = protected)
        indexGPRIn;     % Integer, index of query variable in data
        indexGPROut;    % Integers, indices of regression variable in data
        Data;           % Demonstration data
        xIn;            % Data in for GPR
        xOut;           % Data out for GPR
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
            obj.invK = [];
            obj.xIn = [];
            obj.xOut = [];
        end
        
        function obj = setParam(obj,sigma,W,regulator)
            %setParam Set the hyperparameters of the GP
            %   sigma: scalar: signal variance
            %   W: covariance matrix of Gaussian kernel
            %   regulator: scalar, matrix regulator parameter (optional)
            obj.sigma = sigma;
            obj.W = W;
            if nargin >= 4
                obj.reg = regulator;
            end
        end
        
        function obj = preGPR(obj)
            %preGPR
            obj.xIn = obj.Data(obj.indexGPRIn,:);
            obj.xOut = obj.Data(obj.indexGPROut,:);
            M = pdist2(obj.xIn',obj.xIn');
            K = obj.sigma * exp(-obj.W^-1 * M.^2);
            obj.invK = pinv(K+obj.reg * eye(size(K)));
        end
        
        function [gprOut] = GPR(obj,query)
            %GPR Gaussian process regression
            %   query:1 x N, query points
            %   gprOut: struct, exp.Data and exp.Sigma are regression
            %   data and covariance respectively.
            nDataGPR = size(query,2);   % N
            Md = pdist2(query', obj.xIn');
            Kd = obj.sigma * exp(-obj.W^-1 * Md.^2);
            gprOut.Data = [query; (Kd * obj.invK * obj.xOut')'];
            %Covariance computation
            Mdd = pdist2(query',query');
            Kdd = obj.sigma * exp(-obj.W^-1 * Mdd.^2);
            S = Kdd - Kd * obj.invK * Kd';
            gprOut.Sigma = zeros(obj.nVar-1,obj.nVar-1,nDataGPR);
            for t=1:nDataGPR
                gprOut.Sigma(:,:,t) = eye(obj.nVar-1) * S(t,t);
            end
        end
  
    end
    
    methods (Access = public)
        % Figure
    end
end

