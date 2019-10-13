classdef QuadraticRewardModelProvider < Common.IASObject
    
    properties 
       dataManager 
    end
    
    methods
        function obj = QuadraticRewardModelProvider(dataManager)
            obj = obj@Common.IASObject();
            obj.dataManager = dataManager;
        end

        function rewardArray = createRewardArray(obj, numTimeSteps)
            
            
            rewardStruct.Rss = zeros(obj.dataManager.getNumDimensions('states'), obj.dataManager.getNumDimensions('states'));
            rewardStruct.rs = zeros(obj.dataManager.getNumDimensions('states'), 1);
            rewardStruct.r0 = zeros(1);
            rewardStruct.Raa = zeros(obj.dataManager.getNumDimensions('actions'), obj.dataManager.getNumDimensions('actions'));
            rewardStruct.Rsa = zeros(obj.dataManager.getNumDimensions('states'), obj.dataManager.getNumDimensions('actions'));
            rewardStruct.ra = zeros(obj.dataManager.getNumDimensions('actions'));                        
            
            rewardArray(1:numTimeSteps) = rewardStruct;
    
        end
    
    end
    
    
    methods (Abstract)
        
        [trajectoryReward] = getTrajectoryReward(obj);
        
        [rewardModel]  = getQuadraticRewardModel(obj);        
    end
    
    
end