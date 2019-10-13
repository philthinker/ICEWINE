classdef TransitionFunctionPerturbation < Common.IASObject & Data.DataManipulator
        
    methods
        function obj = TransitionFunctionPerturbation(rootSampler)
            obj = obj@Common.IASObject();
            obj = obj@Data.DataManipulator(rootSampler.getDataManagerForSampler());
            
            obj.addDataManipulationFunction('applyPerturbation', {'phase','states', 'actions'}, {'actions'});
            
        end
    end
    
    methods (Abstract)
         actions = applyPerturbation( obj, phase, states, actions );
    end
    
end