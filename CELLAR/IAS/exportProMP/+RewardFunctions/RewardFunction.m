classdef RewardFunction < Data.DataManipulator
    
    properties
        
    end
    
    methods
        function obj = RewardFunction(dataManager)
            obj = obj@Data.DataManipulator(dataManager);
            
            obj.dataManager.addDataEntry('steps.rewards', 1);
            obj.registerRewardFunction();
        end
        
        function [] = setRewardInputs(obj, varargin)
            obj.setInputArguments('rewardFunction', varargin);
        end
        
        function [] = setRewardOutput(obj, varargin)
            obj.setOutputArguments('rewardFunction', varargin);
        end
        
        function [] = registerRewardFunction(obj)
            obj.addDataManipulationFunction('rewardFunction', {'states', 'actions', 'nextStates'}, {'rewards'});
            obj.addDataFunctionAlias('sampleReward', 'rewardFunction');            
        end
        
    end
    
    methods (Abstract)
        [vargout] = rewardFunction(obj, vargin)
    end
end