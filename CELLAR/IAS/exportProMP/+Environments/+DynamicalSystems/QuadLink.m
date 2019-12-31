classdef QuadLink < Environments.DynamicalSystems.ContinuousTimeDynamicalSystem & Environments.Misc.PlanarForwardKinematics
    
    properties
        masses
        friction
        inertias
        g
        sim_dt      
    end
    
    properties(SetObservable, AbortSet)
        maxTorque = 10;  
    end
    
    methods
        function obj = QuadLink(sampler)
            if (~Common.Settings().hasProperty('usePeriodicStateSpace'))
                Common.Settings().setProperty('usePeriodicStateSpace', 1)
            end
            
            obj = obj@Environments.Misc.PlanarForwardKinematics(sampler.getDataManager(), 4);
            obj = obj@Environments.DynamicalSystems.ContinuousTimeDynamicalSystem(sampler, 4);
            
            
            obj.linkProperty('maxTorque');
            obj.registerPlanarKinematicsFunctions();
            
            obj.lengths 	= [1 1 1 1];
            obj.masses 	= [1 1 1 1];
            obj.friction = [0, 0, 0, 0];
            
            
            obj.dataManager.setRange('states', [-pi, -50, - pi, -50 pi -50 pi -50], [pi, 50, pi, 50 pi 50 pi 50]);
            
            obj.dataManager.setRange('actions', - obj.maxTorque * ones(1,4), obj.maxTorque * ones(1,4));
            obj.dataManager.setRestrictToRange('actions', false); % get rid of restriction for now
            
            
            obj.inertias = obj.masses.*(obj.lengths.^2 + 0.0001)./3.0;
            obj.g 		= 9.81;
            obj.sim_dt 	= 1e-4;
            
            obj.initObject();
        end
        
        
        function [xnew] = getExpectedNextStateContTime(obj, dt, x, action, varargin)
            nSamples = size(x,1);
            
            %            xnew = zeros(size(x));
            %             for i = 1:nSamples
            %
            %                 x_temp = Environments.DynamicalSystems.QuadPendulum_C_ForwardModelWithTorquesMex(x(i,:), action(i,:), dt, ...
            %                     obj.masses, obj.lengths, obj.inertias, obj.g, obj.friction, obj.sim_dt);
            %
            %                 xnew(i,:) = x_temp(1:8);
            %                 % transforming angles to [-pi, pi]
            %             end
            
            x_temp= Environments.DynamicalSystems.QuadPendulum_C_MultipleSamplesForwardModelWithTorquesMex(x', action', dt, ...
                obj.masses, obj.lengths, obj.inertias, obj.g, obj.friction, obj.sim_dt);
            xnew=reshape(x_temp, [12, nSamples])';
            xnew=xnew(:,1:8);     
        end
        
        function [f, f_q, f_u, controlNoise] = getLinearizedContinuousTimeDynamics(obj, x)
            action = zeros(1,4);
            [f_acc, f_q_acc, f_u_acc] = obj.QuadPendulum_C_LinearizedContinuousTimeDynamicsMex(x, action, ...
                obj.masses, obj.lengths, obj.inertias, obj.g, obj.friction);
            
            controlNoise =  eye(obj.dimAction) * obj.Noise_std^2;
            f = zeros(8,1);
            
            f(2:2:end) = f_acc;
            f_q = zeros(8,8);
            f_q(2:2:end, :) = f_q_acc;
            for i = 1:obj.dimAction
                f_q(i * 2 - 1, i * 2) = 1;
            end
            
            f_u = zeros(8,4);
            f_u(2:2:end, :) = f_u_acc;            
        end  
        
        function [MMatrix, offest] = getInvDyn(obj,x)
            assert(length(x) == obj.numJoints*2);
            [MMatrix, offest] = Environments.DynamicalSystems.QuadPendulum_C_InverseModel(x,obj.masses,...
                obj.lengths, obj.inertias, obj.g, obj.friction);
            
        end
                
        
        %%
        function pos = getEndeffectorPos(obj, q)
            error('pst::getEndeffectorPos: deprictated ... use functions of PlanarForwardKinematics instead!\n');
        end
        
    end
    
end
