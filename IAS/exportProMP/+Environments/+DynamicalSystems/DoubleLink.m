classdef DoubleLink < Environments.DynamicalSystems.ContinuousTimeDynamicalSystem & Environments.Misc.PlanarForwardKinematics
    
    properties
        masses
        friction
        inertias
        g
        sim_dt
        PDGains
        PDSetPoints
        
        maxTorque = 50;
    end
    
    methods

        function obj = DoubleLink(sampler)
            
            if (~Common.Settings().hasProperty('usePeriodicStateSpace'))
                Common.Settings().setProperty('usePeriodicStateSpace', 1)
            end
            
            
            obj = obj@Environments.Misc.PlanarForwardKinematics( sampler.dataManager, 2);
            obj = obj@Environments.DynamicalSystems.ContinuousTimeDynamicalSystem(sampler, 2);
            
            
            obj.registerPlanarKinematicsFunctions();
            
            obj.lengths 	= [1 1];
            obj.masses 	= [1 1];
            obj.friction = [0.025, 0.025];
            
            
            obj.dataManager.setRange('states', [-pi -30 -pi -30], [pi 30 pi 30]);
            
            obj.linkProperty('maxTorque');
            obj.dataManager.setRange('actions',-obj.maxTorque*ones(1, 2),obj.maxTorque*ones(1, 2));
            
            
            obj.inertias = obj.masses.*(obj.lengths.^2 + 0.0001)./3.0;
            obj.g 		= 9.81;
            obj.sim_dt 	= 1e-4;
            
            obj.PDSetPoints = 0;
            obj.PDGains = 0;
            
            obj.initObject();

        end
        
        function [xnew, ffwdTorque] = getExpectedNextStateContTime(obj, dt, x, rawaction, varargin)
            % Limiting the torques
            
            xnew = zeros(size(x));
            ffwdTorque = zeros(size(x, 1), 2);
            % Limiting the torques
            minRange = obj.dataManager.getMinRange('actions'); 
            maxRange = obj.dataManager.getMaxRange('actions'); 
            
            action = bsxfun(@max, bsxfun(@min, rawaction, maxRange), minRange);
            
            for i = 1:size(x,1)
                
                x_temp = Environments.DynamicalSystems.DoublePendulum_C_ForwardModelWithTorquesMex(x(i,:), action(i,:), dt, ...
                    obj.masses, obj.lengths, obj.inertias, obj.g, obj.friction, obj.sim_dt, obj.PDSetPoints, obj.PDGains);
                
                ffwdTorque(i,:) = x_temp(5:6);
                xnew(i,:) = x_temp(1:4);
                % transforming angles to [-pi, pi]
            end
            %xnew(:,[1,3]) =  mod(xnew(:,[1,3])+pi, 2*pi)-pi;
            xnew = obj.projectStateInPeriod(xnew);
        end
        
        
        
        function [f, f_q, f_u, controlNoise] = getLinearizedContinuousTimeDynamics(environment, x)
            action = zeros(1,2);
            [f_acc, f_q_acc, f_u_acc] = Environments.DynamicalSystems.DoublePendulum_C_LinearizedContinuousTimeDynamicsMex(x, action, ...
                environment.masses, environment.lengths, environment.inertias, environment.g, environment.friction);
            
            
            controlNoise =  eye(environment.dimAction) * environment.Noise_std^2;
            f = zeros(4,1);
            
            
            f(2:2:end) = f_acc - f_q_acc * x';
            f_q = zeros(4,4);
            f_q(2:2:end, :) = f_q_acc;
            for i = 1:environment.dimAction
                f_q(i * 2 - 1, i * 2) = 1;
            end
            
            f_u = zeros(4,2);
            f_u(2:2:end, :) = f_u_acc;
            
        end
        
    end
    
end
