classdef TransitionFunctionGaussianNoise < Environments.TransitionFunction
    
    properties (GetAccess=public, SetAccess=private)
       
    end
    
    methods
        function obj = TransitionFunctionGaussianNoise( stepDataSampler, dimState, dimAction)
           obj = obj@Environments.TransitionFunction(stepDataSampler, dimState, dimAction);
                       
           obj.addDataManipulationFunction('getExpectedNextState', {'states', 'actions'}, {'nextStates'});
           obj.addDataManipulationFunction('getNoiseCovariance', {'states', 'actions'}, {'systemNoise'});
        end
                      
        
        function [] = setTransitionInputs(obj, varargin)
            obj.setInputArguments('transitionFunction', varargin);
            obj.setInputArguments('getExpectedNextState', varargin);
            obj.setInputArguments('getNoiseCovariance', varargin);
        end
        
        function [] = setTransitionOutput(obj, varargin)
            obj.setOutputArguments('transitionFunction', varargin);
        end                      
    end
    
    methods (Abstract)
        [vargout] = getExpectedNextState(obj, varargin)        
        [vargout] = getSystemNoiseCovariance(obj, varargin)        
    end
end