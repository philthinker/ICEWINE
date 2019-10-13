classdef DynamicalSystem < Environments.TransitionFunctionGaussianNoise
    
    properties(SetObservable,AbortSet)
        Noise_std = 0;
        Noise_mode = 0;
        
        returnControlNoise = false;
        
        usePeriodicStateSpace = 0;
    end
    
    methods
        function obj = DynamicalSystem(stepDataSampler, dimensions)
            
            obj = obj@Environments.TransitionFunctionGaussianNoise(stepDataSampler, dimensions * 2, dimensions);
            
            obj.linkProperty('Noise_std');
            obj.linkProperty('Noise_mode');
            obj.linkProperty('usePeriodicStateSpace');
            
            obj.dataManager.setRange('states', repmat([-pi, -5], 1, dimensions), repmat([pi, 5], 1, dimensions));
            obj.dataManager.setRange('actions', ones(1, dimensions) * -50, ones(1, dimensions) * 50);
            
            if (obj.usePeriodicStateSpace)
                obj.dataManager.setPeriodicity('states', repmat([true,false], 1, dimensions));
            end
            

            obj.dataManager.addDataAlias('jointPositions', 'states', 1:2:(2 * dimensions - 1));
            obj.dataManager.addDataAlias('jointVelocities', 'states', 2:2:(2 * dimensions));
            
            obj.dataManager.addDataAlias('nextJointPositions', 'states', 1:2:(2 * dimensions - 1));
            obj.dataManager.addDataAlias('nextJointVelocities', 'states', 2:2:(2 * dimensions));
                        
            
            obj.addDataManipulationFunction('getControlNoiseStd', {'states', 'actions'}, {'noise_std'});
            obj.addDataManipulationFunction('getControlNoise', {'states', 'actions'}, {'actionsNoise'});
            
            obj.addDataManipulationFunction('getUncontrolledTransitionProbabilities', {'states', 'actions', 'actionsNoise'}, {'logProbTransUC'});
            obj.addDataManipulationFunction('getTransitionProbabilities', {'states', 'actions', 'actionsNoise'}, {'logProbTrans'});
        end
        
        function [] = registerControlNoiseInData(obj)
            obj.dataManager.addDataEntry('steps.actionsNoise', obj.dimAction);
            obj.setTransitionOutput('nextStates', 'actionsNoise');
            obj.returnControlNoise = true;
        end
        
        function [noise] = getControlNoise(obj, x, actions, varargin)
            
            std =  getControlNoiseStd(obj, x, actions, varargin{:});
            
            noise = bsxfun(@times, randn(size(actions)), std);
        end
        
        
        function [controlNoiseStd] = getControlNoiseStd(obj, x, actions, varargin)
            
            %mode = 1;
            %0... additive noise,
            %1... multiplicative noise, else no noise
            
            switch (obj.Noise_mode)
                case 0
                    controlNoiseStd = obj.Noise_std;
                    
                case 1
                    controlNoiseStd = obj.Noise_std * abs(actions);
            end
            
            if (numel(controlNoiseStd) == 1)
                controlNoiseStd = ones(1, obj.dimAction) * controlNoiseStd;
            end
        end
        
        function [logProb] = getTransitionProbabilities(obj, states, actions, noise)
            std =  obj.getControlNoiseStd(states, actions);
            std(std < 10^-8) = 10^-8;
            noiseNorm = bsxfun(@rdivide, noise,  std);
            logProb = - 0.5 * sum(noiseNorm  .^2, 2);
            %prob = log(mvnpdf(noiseNorm, zeros(1, size(std,2)), ones(1,size(std,2))));
        end
        
        function [logProb] = getUncontrolledTransitionProbabilities(obj, states, actions, noise)
            std =  obj.getControlNoiseStd(states, actions);
            std(std < 10^-8) = 10^-8;
                        
            noiseNorm = bsxfun(@rdivide, (actions + noise),  std);
            
            logProb = - 0.5 * sum(noiseNorm  .^2, 2);
%            prob = mvnpdf(noiseNorm, zeros(1, size(std,2)), ones(1,size(std,2)));
        end
        
        function [xnew,  actionNoise] = transitionFunction(obj, x, action, varargin)
            actionNoise = obj.getControlNoise(x, action, varargin);
            
            xnew = obj.getExpectedNextState(x, action + actionNoise, varargin{:});
            xnew = obj.projectStateInPeriod(xnew);
            
%             if (nargout == 2)
%                 [controlNoiseStd] = obj.getControlNoiseStd(x, actions, varargin);
%                 if (all(controlNoiseStd > 0))
%                     transitionProbabilities = -0.5 * actionNoise.^2 - sum(log(controlNoiseStd));
%                 else
%                     transitionProbabilities = ones(size(xnew,1));
%                 end
%             end
        end
        
        function [f, f_q, f_u, controlNoise] = getLinearizedDynamics(obj, states, actions, varargin)
            
            f_q = zeros(obj.dimState, obj.dimState);
            f_u = zeros(obj.dimState, obj.dimAction);
            
            u = zeros(1, obj.dimAction);
            f =  obj.getExpectedNextState(states, u, varargin{:});
            assert(~any(any(isnan(f))));
            stepSize = 0.00001;
            for i = 1:obj.dimState
                qTemp = states;
                qTemp(i) = states(i) + stepSize;
                f1 = obj.getExpectedNextState(qTemp, u, varargin{:});
                qTemp(i) = states(i) - stepSize;
                f2 = obj.getExpectedNextState(qTemp, u, varargin{:});
                f_q(:,i) = (f1 - f2) / (2 * stepSize);
            end
            
            for i = 1:obj.dimAction
                uTemp = u;
                uTemp(i) = u(i) + stepSize;
                f1 = obj.getExpectedNextState(states, uTemp, varargin{:});
                uTemp(i) = u(i) -  stepSize;
                f2 = obj.getExpectedNextState(states, uTemp, varargin{:});
                f_u(:,i) = (f1 - f2) / (2 * stepSize);
            end
            f = f' - f_q * states';
            assert(~any(isnan(f)) && ~any(isnan(f_u(:))) && ~any(isnan(f_q(:))));
            
            if (nargout == 4)
                controlNoise = obj.getControlNoiseStd(states, actions, varargin{:});
                %Q = diag(controlNoise.^2);
            end
        end
        
        function [systemNoise] = getSystemNoiseCovariance(obj, states, actions, varargin)       
            
            [~, ~, B, controlNoise] = obj.getLinearizedDynamics(states, actions, varargin{:});
            systemNoise = B * controlNoise * B';
        end
        
    end
    
    
end


