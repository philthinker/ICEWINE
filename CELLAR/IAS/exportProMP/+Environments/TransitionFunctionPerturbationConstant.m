classdef TransitionFunctionPerturbationConstant < Environments.TransitionFunctionPerturbation
    
    properties(AbortSet, SetObservable)
        phaseMin = 0.0;
        phaseMax = 1.0;
        
        perturbAmp = 0;
    end
    

    methods
        function obj = TransitionFunctionPerturbationConstant(rootSampler)
            obj = obj@Environments.TransitionFunctionPerturbation(rootSampler); 
            
            obj.linkProperty('phaseMin');
            obj.linkProperty('phaseMax');
            obj.linkProperty('perturbAmp');
        end
        
        
        function actions = applyPerturbation( obj, phase, ~, actions )
            idx = (phase < obj.phaseMax) & (phase > obj.phaseMin) ;
            actions(idx) = actions(idx) + obj.perturbAmp;
        end
                            
    end    
    
end