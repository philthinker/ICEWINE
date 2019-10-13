classdef FiniteDifferenceLinearizedModelProvider < Planning.LinearizedModelProvider
    
    properties 
        trajectoryGenerator;
        transitionFunction;
              
    end
    
    properties (SetObservable, AbortSet)
        FiniteDifferenceLinearizationStepSize = 0.00001;
    end
    
    
    methods 
        function [obj] = FiniteDifferenceLinearizedModelProvider(dataManager, transitionFunction)
            obj = obj@Planning.LinearizedModelProvider(dataManager);
            obj.dataManager = dataManager;
            obj.transitionFunction = transitionFunction;
           
            obj.linkProperty('FiniteDifferenceLinearizationStepSize');
        end
        
        function [modelStruct] = getFiniteDifferenceLinearization(obj, state, action, varargin)
            f_q = zeros(obj.transitionFunction.dimState, obj.transitionFunction.dimState);
            f_u = zeros(obj.transitionFunction.dimState, obj.transitionFunction.dimAction);
            
            u = action;
            f =  obj.transitionFunction.getExpectedNextState(state, action, varargin{:});
            assert(~any(any(isnan(f))));
            stepSize = obj.FiniteDifferenceLinearizationStepSize;
            for i = 1:obj.transitionFunction.dimState
                qTemp = state;
                qTemp(i) = state(i) + stepSize;
                f1 = obj.transitionFunction.getExpectedNextState(qTemp, u, varargin{:});
                qTemp(i) = state(i) - stepSize;
                f2 = obj.transitionFunction.getExpectedNextState(qTemp, u, varargin{:});
                f_q(:,i) = (f1 - f2) / (2 * stepSize);
            end
            
            for i = 1:obj.transitionFunction.dimAction
                uTemp = u;
                uTemp(i) = u(i) + stepSize;
                f1 = obj.transitionFunction.getExpectedNextState(state, uTemp, varargin{:});
                uTemp(i) = u(i) -  stepSize;
                f2 = obj.transitionFunction.getExpectedNextState(state, uTemp, varargin{:});
                f_u(:,i) = (f1 - f2) / (2 * stepSize);
            end
            f = f' - f_q * state' - f_u * action';
            assert(~any(isnan(f)) && ~any(isnan(f_u(:))) && ~any(isnan(f_q(:))));
            
            controlNoise = obj.transitionFunction.getControlNoiseStd(state, action, varargin{:});
            systemNoise = f_u * diag(controlNoise.^2) * f_u';
            
            modelStruct.A = f_q;
            modelStruct.B = f_u;
            modelStruct.c = f;
            modelStruct.H = systemNoise;
            
        end
        
    
                
        function [modelArray] = getLinearizedModel(obj, trajectoryPredictor)
            [meanStateTrajectory, meanActionTrajectory] = trajectoryPredictor.getMeanTrajectory();
        
            modelArray = obj.createModelArray(size(meanStateTrajectory,1) - 1);
            for i = 1:size(meanStateTrajectory, 1) - 1
                modelArray(i) =  obj.getFiniteDifferenceLinearization(meanStateTrajectory(i,:), meanActionTrajectory(i,:));
            end
                        
        end
        
    end
    
    
end