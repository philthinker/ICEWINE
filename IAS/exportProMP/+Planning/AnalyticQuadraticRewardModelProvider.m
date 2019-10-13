classdef AnalyticQuadraticRewardModelProvider < Planning.QuadraticRewardModelProvider
    
    properties 
       rewardFunction
    end
    
    methods
        function obj = AnalyticQuadraticRewardModelProvider(dataManager, rewardFunction)
            obj = obj@Planning.QuadraticRewardModelProvider(dataManager);
            obj.rewardFunction = rewardFunction;
        end

        function [trajectoryReward] = getTrajectoryReward(obj, trajectoryPredictor)
            [meanStateTrajectory, meanActionTrajectory] = trajectoryPredictor.getMeanTrajectory();
        
            trajectoryReward = 0;
            for i = 1:size(meanStateTrajectory, 1) - 1
                trajectoryReward = trajectoryReward + obj.getReward(meanStateTrajectory(i,:), meanActionTrajectory(i,:), i);
            end
            trajectoryReward  = trajectoryReward  + obj.finalReward(meanStateTrajectory(end,:));
            
        end
        
        
        function [rewardArray]  = getQuadraticRewardModel(obj, trajectoryPredictor)
            [meanStateTrajectory, meanActionTrajectory] = trajectoryPredictor.getMeanTrajectory();
        
            rewardArray = obj.createRewardArray(size(meanStateTrajectory,1));
            for i = 1:size(meanStateTrajectory, 1)
                [ct H, h, R, r] = obj.rewardFunction.getQuadraticCosts(meanStateTrajectory(i,:), meanActionTrajectory(i,:), i);
        
                rewardArray(i).Rss =  R;
                rewardArray(i).rs =  - 2 * r;
                rewardArray(i).r0 =  ct;
                
                rewardArray(i).Raa =  H;
                rewardArray(i).ra =  h;
                                
            end
            
        end

    end
    
    
    
    
end