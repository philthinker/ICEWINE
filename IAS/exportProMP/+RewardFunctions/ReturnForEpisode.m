classdef ReturnForEpisode < Common.IASObject & Data.DataManipulator
    
    properties
        rewardName;               
        returnName;
    end
    
    methods
        function obj = ReturnForEpisode(dataManager, rewardName, returnName)
           obj = obj@Common.IASObject();
           obj = obj@Data.DataManipulator(dataManager);
           
           if (~exist('rewardName', 'var'))
               rewardName = 'rewards';
           end
           
           if (~exist('returnName', 'var'))
               returnName = 'returns';
           end                      
           
           obj.rewardName = rewardName;
           obj.returnName = returnName;
                      
           level = obj.dataManager.getDataEntryDepth(obj.rewardName) - 1;           
           obj.dataManager.addDataEntryForDepth(level, obj.returnName, 1);
                      
           obj.addDataManipulationFunction('returnFunction', {obj.rewardName}, {obj.returnName}, false);
           obj.addDataFunctionAlias('sampleReturn', 'returnFunction');
           
           finalRewardsName = ['final', upper(obj.rewardName(1)), obj.rewardName(2:end)];
           if (~obj.dataManager.isDataEntry(finalRewardsName))
               obj.setReturnInputs(obj.rewardName);        
           else
               obj.setReturnInputs(obj.rewardName, finalRewardsName);        
           end
        end
        
        function [] = setReturnInputs(obj, varargin)
            obj.setInputArguments('returnFunction', varargin);
        end
        
        function [] = setReturnOutput(obj, varargin)
            obj.setOutputArguments('returnFunction', varargin);
        end                      
    end
    
    methods (Abstract)
        [vargout] = returnFunction(obj, numElements, vargin)        
    end
end