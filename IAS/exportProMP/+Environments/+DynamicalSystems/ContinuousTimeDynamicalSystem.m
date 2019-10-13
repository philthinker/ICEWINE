classdef ContinuousTimeDynamicalSystem < Environments.DynamicalSystems.DynamicalSystem
    
    properties(SetObservable,AbortSet)
        dt = 0.05;
    end
    
    methods
        function obj = ContinuousTimeDynamicalSystem(sampler, dimensions)
            
            obj = obj@Environments.DynamicalSystems.DynamicalSystem(sampler, dimensions);
            
            obj.linkProperty('dt');
            obj.addDataManipulationFunction('getLinearizedContinuousTimeDynamics', {'states', 'actions'}, {'nextStates', 'A', 'B', 'controlNoise'});
        end
        
        
        function [controlNoiseStd] = getControlNoiseStd(obj, x, actions, dt)
            if (nargin < 4)
                dt = obj.dt;
            end
            controlNoiseStd = obj.getControlNoiseStd@Environments.DynamicalSystems.DynamicalSystem(x, actions, dt);
            controlNoiseStd = controlNoiseStd / sqrt(dt);
        end
        
        
        function [varargout] = transitionFunction(obj, x, action, varargin)
            if (obj.returnControlNoise)
                [varargout{1:2}] = obj.transitionFunctionContTime(obj.dt, x, action, varargin{:});
            else
                varargout{1} = obj.transitionFunctionContTime(obj.dt, x, action, varargin{:});
            end
        end
        
        
        function [xnew,  actionNoise] = transitionFunctionContTime(obj, dt, x, action, varargin)
            action = bsxfun(@max, bsxfun(@min, action, obj.maxRangeAction), obj.minRangeAction); 
            actionNoise = obj.getControlNoise(x, action, dt);
            
            xnew = obj.getExpectedNextStateContTime(dt, x, action + actionNoise, varargin{:});
            xnew = obj.projectStateInPeriod(xnew);     
        end
        
        function [nextState] = getExpectedNextState(obj, states, actions, varargin)
            nextState = obj.getExpectedNextStateContTime(obj.dt, states, actions, varargin{:});
        end
        
        function [f, f_q, f_u, controlNoise] = getLinearizedContinuousTimeDynamics(obj, states, action, varargin)
            error('Continuous Time Linear Model is not implemented by subclass!');
        end
    end
    
    methods (Abstract)
        [nextState] = getExpectedNextStateContTime(obj, dt, states, actions, varargin);
    end
end


