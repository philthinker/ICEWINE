classdef ReturnSummedReward < RewardFunctions.ReturnForEpisode
    
    properties
                       
    end
    
    methods
        function obj = ReturnSummedReward( dataManager)
           obj = obj@RewardFunctions.ReturnForEpisode(dataManager);
           
           if (~obj.dataManager.isDataEntry('finalRewards'))
               obj.setReturnInputs('rewards');        
           else
               obj.setReturnInputs('rewards', 'finalRewards');        
           end
        end
        
        
        function [returns] = returnFunction(obj, rewards, finalRewards)                    
            returns = sum(rewards);
            if (exist('finalRewards', 'var'))
                returns = returns + finalRewards;
            end
        end        
    end        
end
