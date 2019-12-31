classdef LinearSystem < Environments.DynamicalSystems.ContinuousTimeDynamicalSystem
    
    properties(SetObservable,AbortSet)
        masses;
        friction;
        maxTorque = 50;
    end
    
    properties
        
        
        A;
        B;
        H;
        
    end
    
    methods
        
        function set.masses(obj,masses)
            obj.masses = masses;
            obj.updateMatrices();
        end
        
        
        function obj = LinearSystem(stepSampler, dimensions)
            obj = obj@Environments.DynamicalSystems.ContinuousTimeDynamicalSystem(stepSampler, dimensions);
            
            obj.masses = ones(1, dimensions);
            obj.friction = zeros(1, dimensions);            
            
            obj.linkProperty('masses');
            obj.linkProperty('friction');
            obj.linkProperty('maxTorque');
            obj.dataManager.setRange('actions',-obj.maxTorque*ones(1, dimensions),obj.maxTorque*ones(1, dimensions));
                        
            obj.B = zeros(obj.dimState, obj.dimAction);
            obj.A = zeros(obj.dimState, obj.dimState);
                         
            obj.initObject();
        end
        
        function [xnew] = getExpectedNextStateContTime(obj, dt, x, action)
            accs = bsxfun(@times, action, 1 ./ obj.masses);
            
            xdot = zeros(size(x));
            xdot(:, 1:2:end) = x(:,2:2:end);
            xdot(:, 2:2:end) = accs;
            
            xnew = x + dt * xdot;
            xnew(:, 1:2:end) = xnew(:, 1:2:end) + accs / 2 * dt ^ 2;
        end
        
        function [H] = getNoiseCovarianceContTime(obj, dt, x, action)
            
            % First scale H matrix with 1/dt² because it contains 2 dts
            % from B, then multiple with timeStep (as it contains 2
            % timeSteps from the new B and 1/timeStep for scaling the noise
            % with the time step
            H = repmat(obj.H * dt,[1 1 size(timeStep,1)]);
        end
        
        function [] = updateMatrices(obj)
            if (~isempty(obj.dimAction))
                obj.B = zeros(obj.dimState, obj.dimAction);
                obj.A = zeros(obj.dimState, obj.dimState);

                for i = 1:obj.dimAction
                    obj.B(i * 2, i) = 1 / obj.masses(i);
                    obj.A(i * 2 - 1, i * 2) = 1;
                end

                obj.H = obj.B * (eye(obj.dimAction) * obj.Noise_std^2) * obj.B';
            end
        end
        
        function [states] = sampleInitState(obj, numElements )
            states = randn(numElements, obj.dimState);
        end
        
        function [obj] = initObject(obj)
            obj.initObject@Environments.DynamicalSystems.ContinuousTimeDynamicalSystem();
            updateMatrices(obj);
        end
        
        function [f, f_q, f_u, controlNoise] = getLinearizedContinuousTimeDynamics(obj, varargin)
            controlNoise =  eye(obj.dimAction) * obj.Noise_std^2;
            f = zeros(obj.dimState, 1);
            f_q = obj.A;
            f_u = obj.B;
        end
        
    end
    
end


