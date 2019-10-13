classdef PhaseGeneratorSpeedProfile < TrajectoryGenerators.PhaseGenerators.PhaseGenerator
    
    properties
        speedprofile;
    end
    
    methods
        
        function obj = PhaseGeneratorSpeedProfile(dataManager, speedprofile)
            obj = obj@TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);
                      
            obj.speedprofile = speedprofile;
%             tmp = cumsum(obj.speedprofile(1:obj.numTimeSteps) * obj.dt);
%             obj.phaseEndTime = tmp(end);
        end
                       
        function [phase] = generatePhase(obj, numElements)
             if (nargin < 2)
                numElements = obj.numTimeSteps;
            end
            phase = cumsum(obj.speedprofile(1:numElements) * obj.dt)' / obj.phaseEndTime;
        end
        
        function [phaseD] = generatePhaseD( obj, numElements )
            if (nargin < 2)
                numElements = obj.numTimeSteps;
            end
            
            phaseD = obj.speedprofile(1:numElements)';
        end
        
        function [phaseDD] = generatePhaseDD( obj, numElements )
            if (nargin < 2)
                numElements = obj.numTimeSteps;
            end
            
            phaseDD = diff(obj.speedprofile(1:numElements));
            
            phaseDD = [phaseDD,phaseDD(end)]';
        end
        
    end        
end
