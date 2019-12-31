classdef InitialStateSampler < Data.DataManipulator
    
    properties
        
    end
    
    methods
        function [obj] = InitialStateSampler(dataSampler)
            
            obj = obj@Data.DataManipulator(dataSampler.getDataManagerForSampler());
            
            obj.addDataManipulationFunction('sampleInitState', {}, {'states'}, true, true);
        end
    
        function [] = registerInitStateFunction(obj)
            obj.setInputArguments('sampleInitState', obj.additionalParameters);            
        end
    end
    
    methods (Abstract)
        
        [states] = sampleInitState(obj, numElements, varargin);
        
    end
end