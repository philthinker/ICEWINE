classdef DMPPhaseGeneratorRhythmic < TrajectoryGenerators.PhaseGenerators.PhaseGenerator
       
    properties (SetObservable, AbortSet)        
    end
    
    methods                
      
        function obj = DMPPhaseGeneratorRhythmic(dataManager)
            obj = obj@TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);

            obj.registerOptionalParameter('Tau', false, 1, 0.1, 0.2, 'parameters');
            obj.registerOptionalParameter('PhaseOffset', false, 1, 0, 1.0, 'parameters');
            obj.setIfNotEmpty('Tau', 1);
            obj.setIfNotEmpty('PhaseOffset', 0); 
            
            obj.registerPhaseFunction();
            obj.setFeatureInputArguments(obj.additionalParameters{:});
        end
        
        function [phase] = generatePhase(obj, numElements, varargin)
            obj.inputParameterDeMux(varargin);
            
            time = ((1:numElements) * obj.dt)';
            phase = obj.generatePhaseFromTime(time, obj.Tau);
        end
        
        function [phaseD] = generatePhaseD( obj, numElements)
            error('PhaseD: Not implemented for DMPs');
        end
        
        function [phaseDD] = generatePhaseDD( obj, numElements)
            error('PhaseDD: Not implemented for DMPs');
        end
        
        function [phase] = generatePhaseFromTime(obj, time, Tau)            
            if (~exist('period', 'var'))
                Tau = obj.Tau;
            end            
            phase = mod(time / Tau + obj.PhaseOffset, 1 + 10^-6);            
        end
        
        function [] = setMetaParametersFromTrajectory(obj, Y)
            obj.Tau = obj.dt * size(Y,1);
        end
        
    end        
end
