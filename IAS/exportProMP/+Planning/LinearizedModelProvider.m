classdef LinearizedModelProvider < Common.IASObject
    
    properties 
        dataManager
    end
    
    methods
        
        function obj = LinearizedModelProvider(dataManager)
            obj = obj@Common.IASObject();
            obj.dataManager = dataManager;
        end
        
        function [modelArray] = createModelArray(obj, numTimeSteps)
            modelStruct.A = zeros(obj.dataManager.getNumDimensions('states'), obj.dataManager.getNumDimensions('states'));
            modelStruct.B = zeros(obj.dataManager.getNumDimensions('states'), obj.dataManager.getNumDimensions('actions'));
            modelStruct.c = zeros(obj.dataManager.getNumDimensions('states'));
            
            modelStruct.H = zeros(obj.dataManager.getNumDimensions('states'), obj.dataManager.getNumDimensions('states'));
            modelArray(1:numTimeSteps) = modelStruct;
        end
        
    end
    
    methods (Abstract)
                
        [model] = getLinearizedModel(obj, trajectoryPredictor);        
        
    end
    
    
end