classdef IsActiveStepSampler < Data.DataManipulator
    %ISACTIVESTEPSAMPLER Samples wether a sequence is still active
    % e.g. whether it is not reset
    
    properties
    end
    
    methods
        function obj = IsActiveStepSampler(dataManager, stepName)
            obj@Data.DataManipulator(dataManager);
            
            if (~exist('stepName', 'var'))
                stepName = 'timeSteps';
            end
            obj.addDataManipulationFunction('isActiveStep', {'nextStates', stepName}, {'isActive'});
            
        end
    end
    
     methods (Abstract) 
         isActiveStep(obj, nextStates, timeSteps)
         toReserve(obj)
     end
    
end

