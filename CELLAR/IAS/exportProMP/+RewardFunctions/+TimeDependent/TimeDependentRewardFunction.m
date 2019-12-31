classdef TimeDependentRewardFunction < RewardFunctions.RewardFunction
    
    properties (AbortSet, SetObservable)
        numTimeSteps;
    end
    
    methods
        function obj = TimeDependentRewardFunction(dataManager)
           obj = obj@RewardFunctions.RewardFunction(dataManager);
           
           level = dataManager.getDataManagerDepth('steps') - 1;
           obj.dataManager.addDataEntryForDepth(level, 'finalRewards', 1);
           
           obj.linkProperty('numTimeSteps');
           obj.registerTimeDependentRewardFunctions();
                                 
        end
        
        function [vargout] = sampleFinalReward(obj, nextStates, timeSteps, varargin)     
            vargout = obj.finalReward(nextStates(end, :), varargin{:});
        end
        
        function [] = registerTimeDependentRewardFunctions(obj)
            obj.setRewardInputs('states', 'actions', 'nextStates', 'timeSteps')            
            obj.addDataManipulationFunction('sampleFinalReward', {'nextStates', 'timeSteps'}, {'finalRewards'}, false);           
        end
        
        
    end
    
    methods (Abstract)
        [vargout] = rewardFunction(obj, states, actions, nextStates, timeSteps) 
        [vargout] = finalReward(obj, finalState, timeStep)
           
    end
end