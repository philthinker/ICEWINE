classdef SamplerInterface < Common.IASObject & Data.DataManipulator
    properties (Access = protected)
        iterIdx = 1;
    end
    
    methods
        function [obj] = SamplerInterface(dataManager)
            obj = obj@Common.IASObject();
            obj = obj@Data.DataManipulator(dataManager);
        end
        
        function [] = setSamplerIteration(obj, iteration)
            obj.iterIdx = iteration;
        end
        
        function [append] = appendNewSamples(obj)
            append = true;
        end
    end
    
    methods (Abstract)
       [] = createSamples(obj, newData, varargin);      
    end
end