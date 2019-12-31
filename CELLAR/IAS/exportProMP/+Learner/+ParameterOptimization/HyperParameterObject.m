classdef HyperParameterObject < Common.IASObject
    
    properties
        
    end
    
    % Class methods
    methods
        function [obj] = HyperParameterObject()
            obj = obj@Common.IASObject();            
        end        
        
        function [params] = getMinHyperParameterRange(obj)
            params = obj.getHyperParameters();
            expParameterTransformMap = obj.getExpParameterTransformMap();
            params(expParameterTransformMap) = params(expParameterTransformMap) * 10^-10;
            params(~expParameterTransformMap) = params(~expParameterTransformMap) * 0.1;            
        end
        
        function [params] = getMaxHyperParameterRange(obj)
            params = obj.getHyperParameters();
            expParameterTransformMap = obj.getExpParameterTransformMap();
            params(expParameterTransformMap) = params(expParameterTransformMap) * 10^10;
            params(~expParameterTransformMap) = params(~expParameterTransformMap) * 10;            
        end
        
        function [expParameterTransformMap] = getExpParameterTransformMap(obj)
            expParameterTransformMap = true(1, obj.getNumHyperParameters());
        end
          
    end
    
    methods (Abstract)
        [numParams] = getNumHyperParameters(obj);
        [] = setHyperParameters(obj, params);
        [params] = getHyperParameters(obj);
        
        
    end
end
