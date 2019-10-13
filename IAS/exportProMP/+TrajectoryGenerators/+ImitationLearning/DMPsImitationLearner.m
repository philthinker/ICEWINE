classdef DMPsImitationLearner < TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner
    
    properties(AbortSet, SetObservable)
        
        useGoalPos;
        useGoalVel;
        useTau;
        useAmplitudeModifier;
        
        useStartPos;
        useStartVel;
    end
    
    properties
    end
    
    methods
        
        function obj = DMPsImitationLearner(dataManager, varargin)
            obj = obj@TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, varargin{:});
            
            obj.linkProperty('useStartPos');
            obj.linkProperty('useStartVel');
            obj.linkProperty('useGoalPos');
            obj.linkProperty('useGoalVel');
            obj.linkProperty('useTau');
            obj.linkProperty('useAmplitudeModifier');
            
        end
             
        function [FF] = getForcingFunction(obj, Y, Yd, Ydd)
            time = (size(Y,1)) * obj.trajectoryGenerator.dt -  (1:size(Y,1))' * obj.trajectoryGenerator.dt;            
            goalTemp = bsxfun(@plus, bsxfun(@times, -time * obj.trajectoryGenerator.getTau(), obj.trajectoryGenerator.GoalVel), obj.trajectoryGenerator.GoalPos);
            
            FF = obj.trajectoryGenerator.betaX .* bsxfun(@minus, goalTemp, Y);
            FF = FF + bsxfun(@minus, obj.trajectoryGenerator.GoalVel, Yd) ./ obj.trajectoryGenerator.getTau();
            FF = bsxfun(@rdivide, Ydd ./ (obj.trajectoryGenerator.getTau().^2) - obj.trajectoryGenerator.alphaX .* FF, obj.trajectoryGenerator.AmplitudeModifier );
        end
        
        function [targetFunction] = getTargetFunctionForImitation(obj, Y, Yd, Ydd)
            forcingFunction = obj.getForcingFunction(Y, Yd, Ydd);
 
            targetFunction = forcingFunction(:);
        end
        
        function [] = setMetaParametersFromTrajectory(obj, Y, Yd, Ydd)
            obj.setMetaParametersFromTrajectory@TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(Y, Yd, Ydd);
            if (obj.setFixedParameters || obj.useStartPos) 
                obj.trajectoryGenerator.StartPos = Y(1,:);
            end
            if (obj.setFixedParameters ||obj.useStartVel) 
                obj.trajectoryGenerator.StartVel = Yd(1,:);
            end

            if (obj.setFixedParameters || obj.useGoalPos)
                obj.trajectoryGenerator.GoalPos  = Y(end,:);    
            end
            
            if (obj.useGoalVel)
                obj.trajectoryGenerator.GoalVel  = Yd(end,:);    
            else
                obj.trajectoryGenerator.GoalVel = zeros(1, size(Y,2));
            end
            
            if (obj.useAmplitudeModifier)
                AmpMod = sign(obj.trajectoryGenerator.GoalPos-obj.trajectoryGenerator.StartPos) .* range(Y,1);
                AmpMod(abs(AmpMod)<1e-6) = 1;                
                obj.trajectoryGenerator.AmplitudeModifier = AmpMod;
            end
        end               
               
    end
    
end