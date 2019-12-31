classdef MeanTrajectoryForwardPlanner < Planning.TrajectoryForwardPlanner
% This class simulates a given feedback controller on a given system
% Only works for traditional transition functions that take state action
% and return nextState!
    
    properties 
        transitionFunction;
        dataManager;
        gainProvider;
        
        stateTrajectory;
        actionTrajectory;
    end
    
    properties (SetObservable, AbortSet)
        numTimeSteps;
    end
    
    methods
        function obj = MeanTrajectoryForwardPlanner(dataManager, transitionFunction)
            
            obj = obj@Planning.TrajectoryForwardPlanner();
            obj.transitionFunction = transitionFunction;
            obj.dataManager = dataManager;
            obj.linkProperty('numTimeSteps');
        end
        
        function [] = updateTrajectories(obj, iteration, gainProvider, startTimeStep, startMu, startSigma)
            numStates =  obj.dataManager.getNumDimensions('states');
            numActions = obj.dataManager.getNumDimensions('actions');

            if (iteration == 1)
                obj.stateTrajectory = repmat(startMu, obj.numTimeSteps + 1, 1);
                obj.actionTrajectory = zeros(obj.numTimeSteps + 1, numActions);                
            else
                obj.stateTrajectory = zeros(obj.numTimeSteps + 1, numStates);
                obj.actionTrajectory = zeros(obj.numTimeSteps + 1, numActions);
                obj.stateTrajectory(startTimeStep,:) = startMu;

                for i = startTimeStep:obj.numTimeSteps
                    [ K, kff] = gainProvider.getFeedbackGainsForT(i);

                    obj.actionTrajectory(i,:) = kff + K * obj.stateTrajectory(i,:)';
                    obj.stateTrajectory(i + 1,:) = obj.transitionFunction.getExpectedNextState(obj.stateTrajectory(i,:), obj.actionTrajectory(i,:));                                 
                end            
            end
        end
        
        function [stateTrajectory, actionTrajectory] = getMeanTrajectory(obj)
            stateTrajectory = obj.stateTrajectory;
            actionTrajectory = obj.actionTrajectory;
        end
    end
            
end