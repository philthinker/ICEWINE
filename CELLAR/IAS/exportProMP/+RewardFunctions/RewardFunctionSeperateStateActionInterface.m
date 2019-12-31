classdef RewardFunctionSeperateStateActionInterface < Common.handleplus   
    properties
                       
    end
    
    methods
        function obj = RewardFunctionSeperateStateActionInterface()
           
        end
        
        function [] = useSeperateStateActionReward(obj, useSeperate)
            if (useSeperate)
                obj.getDataManager().addDataEntry('steps.stateRewards', 1);
                obj.getDataManager().addDataEntry('steps.actionRewards', 1);
                
                obj.setRewardOutput('rewards', 'stateRewards', 'actionRewards');
            else
                obj.setRewardOutput('rewards');
            end
        end      
    end
    
end