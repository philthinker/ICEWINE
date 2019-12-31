classdef TrajectoryTracker < TrajectoryGenerators.TrajectoryTracker.AbstractTrajectoryTracker & Data.OptionalParameterInterface
        
    properties       
       referenceTrajectory
    end
    
    methods
              
        function obj = TrajectoryTracker(dataManager, numJoints, referenceTrajectory) 
            obj = obj@TrajectoryGenerators.TrajectoryTracker.AbstractTrajectoryTracker(dataManager, numJoints); 
            
            if (~exist('referenceTrajectory', 'var'))
                obj.referenceTrajectory = {'referencePos', 'referenceVel', 'referenceAcc'};
            else
                obj.referenceTrajectory = {[referenceTrajectory, 'Pos'], [referenceTrajectory, 'Vel'], [referenceTrajectory, 'Acc']};
            end
            obj.registerTrackingFunction();
        end     
              
        
        function [] = registerTrackingFunction(obj)
            obj.addDataManipulationFunction('getTrackingControl', {'jointPositions', 'jointVelocities', obj.referenceTrajectory{:}, obj.additionalParameters{:}}, {'actions'});
            obj.addDataFunctionAlias('sampleAction','getTrackingControl');
        end 
                                                              
    end        
    
    methods (Abstract)
        [action] = getTrackingControl(obj, jointPositions, jointVelocities, referenceTrajectory, referenceTrajectoryD, referenceTrajectoryDD)
    end
                  
end