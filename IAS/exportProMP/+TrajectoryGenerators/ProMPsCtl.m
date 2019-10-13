classdef ProMPsCtl < TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider & Data.DataManipulator
    
    properties
        trajDistrib
        env
    end
    
    properties(AbortSet, SetObservable)
        
        estimateNoise = true;
        fullNoiseMatrix = false;
        
        numCtl = 0;        
        numTimeSteps = 100;
        dt = 0.01;
        
        A = [];
        B = [];
        c = [];
        SysActionNoise = [];
        
        ctlPinvThresh=1e-8;
        
        stochasticCtl = false;
        linearizeDynamics = false;
        
    end
    
    methods (Static)
        
        function obj = createFromTrial(trial)
            gainProvider = TrajectoryGenerators.ProMPsCtl(trial.dataManager, trial.trajectoryGenerator, trial.transitionFunction);
            obj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(trial.dataManager, trial.numJoints, gainProvider);
        end
    end
    
    
    methods
        
        function obj = ProMPsCtl(dataManager, trajDistrib, env)
            obj@TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider();
            obj@Data.DataManipulator(dataManager);
            
            obj.trajDistrib = trajDistrib;
            obj.env = env;
            obj.numCtl = env.dimAction;
            
            [f, f_q, f_u, obj.SysActionNoise] = env.getLinearizedContinuousTimeDynamics(zeros(1,env.dimState));
            
            obj.c = [ f(1:2:end,1); f(2:2:end,1) ];
            
            obj.A = [ f_q(1:2:end,:); f_q(2:2:end,:) ];
            obj.A = [ obj.A(:,1:2:end), obj.A(:,2:2:end) ];
            
            obj.B = [ f_u(1:2:end,:); f_u(2:2:end,:) ];  
            
            obj.linkProperty('ctlPinvThresh');
            obj.linkProperty('dt');
            obj.linkProperty('numTimeSteps');
            obj.linkProperty('estimateNoise');
            obj.linkProperty('stochasticCtl');
            obj.linkProperty('linearizeDynamics');
            obj.linkProperty('fullNoiseMatrix');
            
        end
        
        %% Interface implementation 
        
        function  [K_t, k_t, Sigma_u]  = getFeedbackGainsForT (obj, data, tms, state)  
            
            if ( obj.linearizeDynamics ) % Linearization on the current mean
                
                [f, f_q, f_u, obj.SysActionNoise] = obj.env.getLinearizedContinuousTimeDynamics(mean(state,1));
                obj.c = [ f(1:2:end,1); f(2:2:end,1) ];                
                obj.A = [ f_q(1:2:end,:); f_q(2:2:end,:) ];
                obj.A = [ obj.A(:,1:2:end), obj.A(:,2:2:end) ];                
                obj.B = [ f_u(1:2:end,:); f_u(2:2:end,:) ];
                
            end
            
            
            if tms < obj.numTimeSteps
                [mu_t_n_t1, Sigma_t_n_t1 ] = obj.trajDistrib.callDataFunctionOutput('getExpectationAndSigma',data, 1, [tms(1), tms(1)+1]);
                mu_t = mu_t_n_t1(1:2:end,:);
                Sigma_t = Sigma_t_n_t1(1:2:end,1:2:end);
                Sigma_t1 = Sigma_t_n_t1(2:2:end,2:2:end);
                Sigma_t_t1 = Sigma_t_n_t1(1:2:end,2:2:end);
                
%                 tmp = Sigma_t1 - Sigma_t_t1' / Sigma_t * Sigma_t_t1;
                tmp = Sigma_t1 - Sigma_t_t1' * pinv( Sigma_t, obj.ctlPinvThresh) * Sigma_t_t1;
            else
                [mu_t, Sigma_t ] = obj.trajDistrib.callDataFunctionOutput('getExpectationAndSigma',data, 1, tms(1));
                tmp = zeros(obj.env.dimState);                
            end
            
           
            if ( obj.fullNoiseMatrix)
                Sigma_sdt_t = zeros(size(tmp));
                idx = obj.env.dimState/2+1;
                Sigma_sdt_t(idx:end,idx:end) = tmp(idx:end,idx:end); % TODO some hack is required to remove sys noise!
                                   % should enforce [ 0  0 0 hat(B) Sig
                                   % hat(B)'?
                                   % fix the estimate noise option
            else
                tmp = diag(tmp);
                tmp = tmp(length(tmp)/2+1:end);
                Sigma_sdt_t = diag ( [zeros(obj.env.dimState - obj.numCtl,1);
                    obj.boundToSysNoise(tmp)] );
            end
            
                        
            [mu_td, Sigma_td_half] = obj.trajDistrib.callDataFunctionOutput('getStateDistributionD',data, 1, tms(1));
            [K_t, k_t] = obj.getCtlGains (mu_t, mu_td, Sigma_t, Sigma_td_half, Sigma_sdt_t);
            
            % Sigma_sdt = B * dt * Sigma_u/dt * B' * dt
            % Sigma_u = B-1 Sigms_s B-T / dt
            % Sigma_u / dt = B-1 Sigms_s B-T / dt^2 (to scale to timestep)
            Sigma_u = (obj.B \ Sigma_sdt_t / obj.B') / obj.dt^2 ...
                - obj.SysActionNoise / obj.dt ;
            Sigma_u = (Sigma_u + Sigma_u' ) * 0.5;
            
            Kp_t = K_t(:,1:obj.trajDistrib.numJoints);
            Kd_t = K_t(:,(1:obj.trajDistrib.numJoints)+obj.trajDistrib.numJoints);  
                                        
            % convert to pos vel pos vel notation            
            K_t = zeros(size(Kp_t,1), 2* size(Kp_t,2));
            K_t(:, 1:2:end) = Kp_t;
            K_t(:, 2:2:end) = Kd_t;
                                                                                
           if ( ~obj.stochasticCtl)
               Sigma_u = zeros(size(Sigma_u));
           end
           
        end
        
        %%  
        
        function Sigma_sdtNew_diag = boundToSysNoise (obj,Sigma_sdt_diag)
            
            Bhat = obj.B(obj.trajDistrib.numJoints+1:end,:);
          
            %  Bhat * dt * SysActionNoise/dt * Bhat' * dt 
            SysNoise = Bhat * obj.SysActionNoise * Bhat' .* obj.dt;
            
            % Sigma_sdt = Bhat * dt * Sigma_u/dt * Bhat' * dt 
            
            if (obj.estimateNoise)
                Sigma_sdtNew_diag = max(0, Sigma_sdt_diag - diag(SysNoise) );
                Sigma_sdtNew_diag = Sigma_sdtNew_diag + diag(SysNoise);
            else
                Sigma_sdtNew_diag = diag(SysNoise);
            end
           
        end
        
        
        function [K_t, k_t] = getCtlGains (obj, mu_x, mu_xd, Sigma_t, Sigma_td_half, Sigma_u)
            
            % M_temp = Sigma_td_half;
            % M_temp = (M_temp + M_temp') / 2;
            % M_temp(obj.numJoints+1:end,obj.numJoints+1:end) = M_temp(obj.numJoints+1:end,obj.numJoints+1:end)';
            M = Sigma_td_half - obj.A * Sigma_t - 0.5 * Sigma_u / obj.dt;
            
            % M(obj.numJoints+1:end,obj.numJoints+1:end) = M(obj.numJoints+1:end,obj.numJoints+1:end)';
            % M(obj.numJoints+1:end,obj.numJoints+1:end) = 0.5*(M(obj.numJoints+1:end,obj.numJoints+1:end)'+M(obj.numJoints+1:end,obj.numJoints+1:end));
            
            % M(obj.numJoints+1:end,obj.numJoints+1:end) = tril(...
            %  (M(obj.numJoints+1:end,obj.numJoints+1:end)'+M(obj.numJoints+1:end,obj.numJoints+1:end)))...
            %  -diag(diag(M(obj.numJoints+1:end,obj.numJoints+1:end)));
            
%             K_t = obj.B \ M / Sigma_t;
            K_t = (obj.B \ M ) * pinv(Sigma_t, obj.ctlPinvThresh);
            
            % K_t(:,obj.numJoints+1:end) = K_t(:,obj.numJoints+1:end)';
            
            % B_hat = B(obj.trajDistrib.numJoints+1:end, :);
            % tmp = mu_xd-(A+B*K_t)*mu_x-c;
            % k_t = B_hat \ tmp(obj.trajDistrib.numJoints+1:end, :);
            % k_t = B_hat \ [ zeros(obj.trajDistrib.numJoints), eye(obj.trajDistrib.numJoints) ] * (mu_xd-(A+B*K_t)*mu_x-c);
            k_t = obj.B \ (mu_xd-(obj.A+obj.B*K_t)*mu_x-obj.c);  %faster than the prev two ways
            
        end
 
    end

end
