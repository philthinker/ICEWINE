classdef ParametricModel < Functions.MappingInterface
    
    properties 
    end
    
    methods
        function obj = ParametricModel()
            obj = obj@Functions.MappingInterface();
        end        
        
        function [] = registerGradientModelFunction(obj)            
            
            obj.addDataManipulationFunction('getLikelihoodGradient', {obj.inputVariables{:}, obj.outputVariable}, [obj.outputVariable, 'GradLike']);
        end
       

        function Fim = getFisherInformationMatrix(obj)
            warning('policysearchtoolbox: Fisher Information Matrix not implemented');
            Fim = zeros(obj.numParameters, obj.numParameters);
        end
        
        function Fim = getLikelihoodGradient(obj)
            warning('policysearchtoolbox: Likelihood Gradient not implemented');
            gradient = zeros(obj.numParameters, 1);
        end
    end
        
end