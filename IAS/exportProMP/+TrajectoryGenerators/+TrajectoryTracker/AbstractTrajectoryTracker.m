classdef AbstractTrajectoryTracker < Data.DataManipulator
    
    properties(AbortSet, SetObservable)
        numJoints
    end
    
    methods
        
        function obj = AbstractTrajectoryTracker(dataManager, numJoints)
            obj = obj@Data.DataManipulator(dataManager);
            obj.numJoints = numJoints;
            
            %obj.registerTrackingFunction();
        end
        
    end
    
    methods(Abstract)
         [] = registerTrackingFunction(obj)
    end
    
    
end