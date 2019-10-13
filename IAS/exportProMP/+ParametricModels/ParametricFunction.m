classdef ParametricFunction < Functions.MappingInterface
    
    properties 
        
    end
    
    methods
        function obj = ParametricFunction()
            obj = obj@Functions.MappingInterface();            
        end        
        
        function [] = registerGradientFunction(obj)
            if (isempty(obj.inputVariables) || ~isnumeric(obj.inputVariables{1}))
                obj.addDataManipulationFunction('getGradient', obj.inputVariables, [obj.outputVariable, 'Grad']);
            end
        end
        
        function [] = registerGradientDataEntry(obj)
            obj.dataManager.addDataEntry([obj.outpurVariable, 'Grad'], obj.numParameters);
        end
             
        
    end
    
    methods (Abstract)
        [gradient] = getGradient(obj, varargin);
        
        [numParameters] = getNumParameters(obj);
        
        [] = setParameterVector(obj, theta);
        [theta] = getParameterVector(obj);
    end
end