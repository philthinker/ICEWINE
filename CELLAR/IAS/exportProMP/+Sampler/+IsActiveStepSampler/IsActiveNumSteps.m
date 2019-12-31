classdef IsActiveNumSteps < Sampler.IsActiveStepSampler.IsActiveStepSampler
    % IsActiveNumSteps is active until numSteps is reached
    
    properties(SetObservable,AbortSet)
        numTimeSteps = 40;
        resetProb 
        resetProbName
    end
    
    methods
        function obj = IsActiveNumSteps(dataManager, nameSteps)
            % @param dataManager Data.DataManager this sampler operates on
            % @param nameSteps name of the steps (default: 'timeSteps')
            if (~exist('nameSteps', 'var'))
                nameSteps = 'timeSteps';
            end

            obj@Sampler.IsActiveStepSampler.IsActiveStepSampler(dataManager, nameSteps);
            obj.linkProperty('numTimeSteps', ['num', upper(nameSteps(1)), nameSteps(2:end) ]);
            
            
            obj.resetProb = 1/ obj.numTimeSteps;
            
            obj.resetProbName = ['resetProb', upper(nameSteps(1)), nameSteps(2:end)];
            obj.linkProperty('resetProb', obj.resetProbName );
        end
        
        function isActive = isActiveStep(obj, ~, timeSteps)
            isActive = timeSteps < obj.numTimeSteps;
        end
        
        function numTimeSteps = toReserve(obj)
            numTimeSteps = obj.numTimeSteps;
        end
        
        function [] = setNumTimeSteps(obj,steps)
            obj.numTimeSteps = steps;
        end
    end
    
end

