classdef DMPPhaseGenerator < TrajectoryGenerators.PhaseGenerators.PhaseGenerator 
       
    properties (SetObservable, AbortSet)        
        alphaZ 
        Tau = 1;
    end
    
    methods                
      
        function obj = DMPPhaseGenerator(dataManager)
            obj = obj@TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);

            obj.registerOptionalParameter('Tau', false, 1, 0.8, 1.2, 'parameters');
            obj.setIfNotEmpty('useTau', false);

            obj.linkProperty('alphaZ');           
            obj.setIfNotEmpty('alphaZ', 2 / (obj.numTimeSteps * obj.dt * obj.Tau));
                        
            obj.registerPhaseFunction();
            obj.setFeatureInputArguments(obj.additionalParameters{:});
        end
        
        function [phase] = generatePhase(obj, numElements,  varargin)
            obj.inputParameterDeMux(varargin);
            
            time = ((1:numElements) * obj.dt)';
            phase = obj.generatePhaseFromTime(time, obj.Tau);
        end
        
        function [phaseD] = generatePhaseD( obj, numElements )
            error('PhaseD: Not implemented for DMPs');
        end
        
        function [phaseDD] = generatePhaseDD( obj, numElements )
            error('PhaseDD: Not implemented for DMPs');
        end
        
        function [phase] = generatePhaseFromTime(obj, time, tau)            
            if (~exist('tau', 'var'))
                tau = obj.Tau;
            end
            time = time .* tau;
            phase = exp(-obj.alphaZ* time);            
        end
        
        function [] = setMetaParametersFromTrajectory(obj, Y)
            obj.setMetaParametersFromTrajectory@TrajectoryGenerators.PhaseGenerators.PhaseGenerator(Y);
            obj.alphaZ = 2 / (obj.numTimeSteps * obj.dt * obj.Tau);
        end
        
    end        
end
