classdef CovarianceFbCtl < TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider & Data.DataManipulator
    
    properties
        trajDistrib
        env
        numCtl = 0;
    end
    
    properties(AbortSet, SetObservable)
        useConditioning = false;
        useVelFF = true;
        scaleKp = 0.4;
        scaleKd = 0.4;
        estimateVelFB = false;
        useFbEigDecomp = false;
    end
    
    methods (Static)
        
        function obj = createFromTrial(trial)
            gainProvider = TrajectoryGenerators.CovarianceFbCtl(trial.dataManager, trial.trajectoryGenerator, trial.transitionFunction);
            obj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(trial.dataManager, trial.numJoints, gainProvider);
        end
    end
    
    
    methods
        
        function obj = CovarianceFbCtl(dataManager, trajDistrib, env)
            obj@TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider();
            obj@Data.DataManipulator(dataManager);
            
            obj.trajDistrib = trajDistrib;
            obj.env = env;
            obj.numCtl = env.dimAction;
            
            obj.linkProperty('useConditioning');
            obj.linkProperty('useVelFF');
            obj.linkProperty('scaleKp');  
            obj.linkProperty('scaleKd');
            obj.linkProperty('estimateVelFB');
            obj.linkProperty('useFbEigDecomp');
            
        end
        
        %% Interface implementation
        
        function  [K_t, k_t, Sigma_u]  = getFeedbackGainsForT (obj, data, tms, state)
            
            idx = 1:obj.trajDistrib.numJoints;
            
            if ( obj.useConditioning )
                
                % conditionTrajectory(obj, timepoint, y, sigmaVector, mask)
                
                % Use conditioning on the current state
                % newModel = obj.pmp.conditionTrajectory(timePoint, state(idx), 0.00001,[1;0]);
                % [~, ~, Sigma_t] = newModel.getDistributionsForTimePoint(context, timePoint);
                
                %
                %  K_d = inv(Sigma_t(idx_vel,idx_vel));
                
                %  newModel = obj.pmp.conditionTrajectory(timePoint, state(idx_vel), 0.00001,[0;1]);
                %  [~, ~, Sigma_t] = newModel.getDistributionsForTimePoint(context, timePoint);
                %
                %  K_p = inv(Sigma_t(idx,idx));
                
                %  K_d = 2*sqrt(K_p);
                keyboard;
            else
                [mu_t, Sigma_t ] = obj.trajDistrib.callDataFunctionOutput('getExpectationAndSigma',data, 1, tms(1));
            end
            
            if ( obj.useFbEigDecomp )
                
                % and all steps...
                K_p_max = 300; %TODO make param
                K_p_min = 0.0;%TODO make param
                
                [V,D] = eig(inv(Sigma_t(idx,idx)));
                
                eigVal = diag(D);
                if max(eigVal) ~= min(eigVal)
                    eigValNorm = (eigVal - min(eigVal)) / (max(eigVal) - min(eigVal));
                else
                    eigValNorm = eigVal / max(eigVal);
                end
                D = diag(K_p_min + ( K_p_max - K_p_min ) * eigValNorm);
                Kp_t = V*D/V;
                
            else
                Kp_t = Sigma_t(idx,idx) \ obj.scaleKp;
            end
            
            
            if ( obj.estimateVelFB)
                Kd_t = Sigma_t((idx+1):end,(idx+1):end) \ obj.scaleKd;
            else
                Kd_t = 2*sqrt(Kp_t);
            end
            
            
            
            if ( obj.useVelFF )
                k_t = [Kp_t, Kd_t] * mu_t;
            else
                k_t = Kp_t * mu_t(idx);
            end
            
            Sigma_u = zeros(obj.numCtl,1);
            
            % convert to pos vel pos vel notation
            K_t = zeros(size(Kp_t,1), 2* size(Kp_t,2));
            K_t(:, 1:2:end) = - Kp_t;
            K_t(:, 2:2:end) = - Kd_t;
            
        end
        
    end
    
end
