classdef OMCPerformance
    %OMCPerformance Performance of the optical motion capture platforms
    %   It is used to deal with the test data for several indicators and we
    %   assume that all data are ROW vectors, i.e. x = [x1,x2,x3].
    %   
    %   Haopeng Hu
    %   2019.12.29
    %   All rights reserved
    %   
    %   Notations:
    %   |   N:  num. of data samples
    %   |   D:  dim. of the data
    %   |   M:  num. of the sample sets
    
    properties (Access = public)
        % Indivdual Tags
        id;             % ID
        name;           % Name of the test experiment
        position;       % Position of test experiment
        velocity;       % Velocity of test experiment
        acceleration;   % Acceleration of test experiment
        % Static performance
        SD;             % Standard deviation
        MaxAE;          % Maximum absolute error
        MAE;            % Mean absolute error
        RMSE;           % Root mean squared error
        Sensitivity;    % Sensitivity performance
        % Dynamic performance
        trajectoryR;        % Reference trajectory of test experiment
        ContourPerform;     % Contour performance
        PointLossPerform;   % Point loss performance
    end
    
    methods
        function obj = OMCPerformance(ID,position,name)
            %OMCPerformance Init. the performance
            %   name: String, name of the experiment (optional)
            %   position: 1 x 3, position of the experiment
            if nargin < 2
                name = [];
            end
            % ID
            obj.id = floor(ID);
            obj.name = name;
            obj.position = position;
            obj.velocity = [0.0, 0.0, 0.0];
            obj.acceleration = [0.0, 0.0, 0.0];
            obj.trajectoryR = [];
            % Performance
            obj.SD = [];
            obj.MAE = [];
            obj.MaxAE = [];
            obj.RMSE = [];
            obj.Sensitivity = [];
            obj.ContourPerform.Linear = [];
            obj.ContourPerform.Circle = [];
            obj.PointLossPerform.Linear = [];
            obj.PointLossPerform.Circle = [];
        end
        
        function obj = setVelocity(obj,velocity)
            %setVelocity Set the test velocity property
            %   velocity: scalar, note that it can be a linear velocity or
            %   angular velocity
            obj.velocity = velocity;
        end

        function obj = staticPerformance(obj,StaticData)
            %staticPerformance Output static performance once and for all.
            %If there are more than one StaticData structs, the mean value
            %of each performance will be calculated.
            %   StaticData: 1 x M struct array, the data sets of static
            %   performance test, where
            %   StaticData.hatXs: N1 x D, data to compute SD
            %   StaticData.hatX: N2 x D, data to test accuracy
            %   StaticData.X: N2 x D, reference data for accuracy
            M = length(StaticData);
            tmp = zeros(4,M);
            for i = 1:M
                tmp(1,i) = obj.SDCompute(StaticData.hatXs);
                tmp(2,i) = obj.MaxAE(StaticData.hatX,StaticData.X);
                tmp(3,i) = obj.MAECompute(StaticData.hatX,StaticData.X);
                tmp(4,i) = obj.RMSEComptue(StaticData.hatX,StaticData.X);
            end
            obj.SD = mean(tmp(1,:));
            obj.MaxAE = mean(tmp(2,:));
            obj.MAE = mean(tmp(3,:));
            obj.RMSE = mean(tmp(4,:));
        end
        function obj = dynamicPerformance(obj,DynamicData)
            %dynamicPerformance Output dynamic performance once and for all
            %If there are more than one DtaticData structs, the mean value
            %of each performance will be calculated. We assume that all the
            %lost points are [].
            %   DynamicData: 1 x N struct array, the data sets of dynamic
            %   performance test, where
            %   DynamicData.Sensitivity: scalar, the sensitivity
            %   DynamicData.hatXXl: N1 x D, data of a linear trajectory
            %   DynamicData.XXl: N2 x D, reference of the linear trajectory
            %   DynamicData.hatXXc: N3 x D, data of a circle trajectory
            %   DynamicData.XXc: N4 x D, reference of the circle trajectory
            M = length(DynamicData);
            tmp = zeros(5,M);
            for i = 1:M
                tmp(1,i) = DynamicData.Sensitivity;
                tmp(2,i) = obj.ContourPerformCal(DynamicData.hatXXl,DynamicData.XXl);
                tmp(3,i) = obj.ContourPerformCal(DynamicData.hatXXc,DynamicData.XXc);
                tmp(4,i) = obj.PointLossPerformCal(DynamicData.hatXXl);
                tmp(5,i) = obj.PointLossPerformCal(DynamicData.hatXXc);
            end
            obj.Sensitivity = mean(tmp(1,:));
            obj.ContourPerform.Linear = mean(tmp(2,:));
            obj.ContourPerform.Circle = mean(tmp(3,:));
            obj.PointLossPerform.Linear = mean(tmp(4,:));
            obj.PointLossPerform.Circle = mean(tmp(5,:));
        end
    end
    
    methods (Access = public)
        % Static Performance
        function SD = SDCompute(obj,hatX)
            %SDCompute Compute the SD.
            %   X: N x D, static data to test the SD. Note that we always
            %   assume that the data are ROW vectors (covectors) for
            %   efficiency.
            mu = mean(hatX,1)';
            N = size(hatX,1);
            hatX = hatX';
            SD = sqrt(trace((hatX-mu)'*(hatX-mu))/N);
        end
        function MaxAE = MaxAECompute(obj,hatX,X)
            %MaxAECompute Compute the MaxAE
            %   hatX: N x D, static data to test the MAE
            %   X: N x D, reference data
            E = (hatX - X)';
            MaxAE = sqrt(max(diag(E'*E)));
        end
        function MAE = MAECompute(obj,hatX,X)
            %MAECompute Compute the MAE
            %   hatX: N x D, static data to test the MAE
            %   X: N x D, reference data
            E = (hatX - X)';
            MAE = mean(sqrt(diag(E'*E)));
        end
        function RMSE = RMSEComptue(obj,hatX, X)
            %RMSECompute Compute the RMSE
            %   hatX: N x D, static data to test the RMSE
            %   X: N x D, reference data
            E = (hatX - X)';
            N = size(hatX,1);
            RMSE = sqrt(trace(E'*E)/N);
        end
        function SensiSteps = SensitivityStepGen(obj,mode)
            %SensitivityStepGen Generate test steps for sensitivity
            %   mode: integer, 0 for equal difference steps (default), 1
            %   for equal ratio steps (1.5)
            if nargin < 2
                mode = 0;
            end
            SensiSteps = zeros(1,20);
            if mode == 0
                % Equal difference steps
                SensiSteps = (1:20)*obj.SD;
            elseif mode == 1
                % Equal ratio steps
                SensiSteps = obj.SD.*(1.5.^(0:19));
            end
        end
        % Dynamic performance
        function [ContourPerform,XX1] = ContourPerformCal(obj,XX,XXR)
            %ContourPerformCal Calculate the contour performance. We assume
            %that the lost entries are all [].
            %   XX: N x 3, linear or circle trajectory
            %   XXR: Nr x 3, linear or circle reference trajectory
            %   Contour: scalar, the performance
            %   XX1: N x 3, trajectory without lost entries.
            ContourPerform = 0;    N = size(XX,1);  
            beta = 2;   % Coefficient (You can modify it for your convenience)
            lostLogicIndices = isnan(XX(:,1));
            % Lost points are omitted
            XX1 = XX(lostLogicIndices==0,:);
            for i = 1:N
                ContourPerform = ContourPerform + exp(beta * obj.absTrajError(XX1(i,:),XXR))-1;
            end
            ContourPerform = ContourPerform/N;
        end
        function PointLossPerform = PointLossPerformCal(obj,XX)
            %PointLossPerformCal Calculate the point loss performance. We
            %assume that the lost entries are all [].
            %   XX: N x 3, data trajectory
            %   PointLossPerform: scalar, the performance
            PointLossPerform = 0;   N = size(XX,1);
            alpha = 2;  % Coefficient (You can modify it for your convenience)
            lostLogicIndices = isnan(XX(:,1));
            t = 0;
            for i = 1:N
                if lostLogicIndices(i)
                    t = t + 1;
                    PointLossPerform = PointLossPerform + exp(alpha*(t-1));
                else
                    t = 0;
                end
            end
            PointLossPerform = PointLossPerform/N;
        end
    end
    
    methods (Access = protected)
        e = absContourError(obj,x,XR);          % Absolute contour error
    end
end

