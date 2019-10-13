classdef PhaseGenerator < FeatureGenerators.FeatureGenerator & Data.OptionalParameterInterface & Learner.Learner
    
    properties (SetObservable, AbortSet)
        dt = 0.01;
        phaseEndTime = 1.0;
        numTimeSteps = 100;
        phaseOffset = 0;
    end
    
    
    methods
        
        function obj = PhaseGenerator(dataManager, phaseName, phaseOffset)
            if (~exist('phaseName', 'var'))
                phaseName = '~phase';
            end
            
            obj = obj@FeatureGenerators.FeatureGenerator(dataManager, 'steps', phaseName, 1, 1);
            obj = obj@Data.OptionalParameterInterface();
            
            if (~exist('phaseOffset', 'var'))
                phaseOffset = 0;
            end
            obj.phaseOffset = phaseOffset;
            
            obj.linkProperty('dt');
            obj.linkProperty('numTimeSteps');
            obj.phaseEndTime = obj.dt * (obj.numTimeSteps + abs(obj.phaseOffset));
            obj.linkProperty('phaseEndTime');
            
            obj.setIsPerEpisodeFeatureGenerator();
            dataManager.addDataEntry( 'steps.phaseD',  1 );
            dataManager.addDataEntry( 'steps.phaseDD', 1 );
            obj.registerPhaseFunction();
        end
        
        function [] = registerPhaseFunction(obj)
            obj.addDataManipulationFunction('generatePhase',  obj.additionalParameters, {'phase'},  Data.DataFunctionType.PER_EPISODE, true );
            obj.addDataManipulationFunction('generatePhaseD', obj.additionalParameters, {'phaseD'}, Data.DataFunctionType.PER_EPISODE, true );
            obj.addDataManipulationFunction('generatePhaseDD',obj.additionalParameters, {'phaseDD'},Data.DataFunctionType.PER_EPISODE, true );
        end
        
        function [phase] = generatePhaseFromTime( obj, time )
            phase = time / obj.phaseEndTime;
        end
        
        function [phase] = generatePhase( obj, numElements )
            if (nargin < 2)
                numElements = obj.numTimeSteps;
            end
            
            startT = 1;
            endT = numElements;
            if (obj.phaseOffset>0)
                startT = startT + obj.phaseOffset;
                endT = endT + obj.phaseOffset;
            end
            
            time = (startT:endT)' * obj.dt;
            phase = obj.generatePhaseFromTime(time);
        end
        
        function [phaseD] = generatePhaseD( obj, numElements )
            if (nargin < 2)
                numElements = obj.numTimeSteps;
            end
            
            phaseD = ones(numElements,1) / obj.phaseEndTime;
        end
        
        function [phaseDD] = generatePhaseDD( obj, numElements )
            if (nargin < 2)
                numElements = obj.numTimeSteps;
            end
            
            phaseDD = zeros(numElements,1);
        end
        
        function [] = setMetaParametersFromTrajectory(obj, Y)
            obj.numTimeSteps = length(Y);
            obj.phaseEndTime = obj.dt * obj.numTimeSteps;
        end
        
        function [phase] = getFeaturesInternal(obj, numElements, varargin)
            phase = obj.generatePhase(numElements, varargin{:});
        end
        
    end
end

