classdef TrajectoryForwardPlanner < Common.IASObject
    
    properties 
        
    end
    
    methods (Abstract)
        
        [] = updateTrajectories(obj, iteration, gainProvider, startTimeStep, startMu, startSigma);
        [stateTrajectory, actionTrajectory] = getMeanTrajectory(obj);
    end
    
    
end