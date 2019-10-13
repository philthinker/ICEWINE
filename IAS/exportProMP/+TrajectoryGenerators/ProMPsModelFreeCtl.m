classdef ProMPsModelFreeCtl < TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider & Data.DataManipulator & Learner.Learner
    
    properties
        trajDistrib
        numCtl = 0;        

    end
    
    properties(AbortSet, SetObservable)
        
        estimateNoise = true;        
        fullNoiseMatrix = false;
        stochasticCtl = false;
        
        numTimeSteps = 100;
        dt = 0.01;
        
        Kp_t = [];
        Kd_t = [];
        k_t = [];
        Sigma_ut = [];        

        SysActionNoise = [];
        Noise_std;
        
        regularizationFactorModelFreeProMPCtrl = 0;
    end
    
    methods
        
        function obj = ProMPsModelFreeCtl(dataManager, trajDistrib)
            obj@TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider();
            obj@Data.DataManipulator(dataManager);
            
            obj.trajDistrib = trajDistrib;
            
            obj.numCtl = obj.trajDistrib.numJoints - obj.trajDistrib.numObs ;
            
            obj.linkProperty('dt');
            obj.linkProperty('numTimeSteps');
            obj.linkProperty('estimateNoise');
            obj.linkProperty('stochasticCtl');
            obj.linkProperty('Noise_std');
            obj.linkProperty('regularizationFactorModelFreeProMPCtrl');
            obj.unlinkProperty('regularizationFactorModelFreeProMPCtrl');
            
            
            obj.SysActionNoise = eye(obj.numCtl) * obj.Noise_std^2;          
            
            obj.addDataManipulationFunction('getFeedbackGains', {}, ...
                      {''}, Data.DataFunctionType.PER_EPISODE, false );                                                        
            obj.setTakesData('getFeedbackGains', true);
            
            obj.addDataManipulationFunction('updateModel', {}, ...
                      {}, Data.DataFunctionType.ALL_AT_ONCE, false );                                                        
            obj.setTakesData('updateModel', true);
            
            
        end
        
        %% Interface implementation 
        
        function [] =  updateModel ( obj, data)  
            [ obj.Kp_t, obj.Kd_t, obj.k_t, obj.Sigma_ut ] = ...
                                   obj.getFeedbackGains ( data, 1, :); %basis the same
        end
        
        function  [K_t, k_t, Sigma_u]  = getFeedbackGainsForT (obj, tms)
            
            idxPos = ((1:obj.numCtl)-1)*obj.numTimeSteps;
            
            t = tms(1); %TODO
            
            Kp_t = obj.Kp_t(idxPos+t,:);
            Kd_t = obj.Kd_t(idxPos+t,:);
            k_t  = obj.k_t(idxPos+t); 
            
            if ( obj.stochasticCtl)
                Sigma_u = obj.Sigma_ut(idxPos+t,:);                
            else
                Sigma_u = zeros(size(obj.Sigma_ut(idxPos+t,:)));
            end            
               
            K_t = zeros(size(Kp_t,1), 2* size(Kp_t,2));
            K_t(:, 1:2:end) = Kp_t;
            K_t(:, 2:2:end) = Kd_t;
        end
            
        
        function [Kp_t, Kd_t, k_t, Sigma_u] = getFeedbackGains (obj, data, varargin)
            
            [mu_t, Sigma_t ] = obj.trajDistrib.callDataFunctionOutput('getExpectationAndSigma',data, varargin{:});
            
            [K_t, k_t, Sigma_u] = obj.getCtlGains ( mu_t, Sigma_t );  
            
            nJoints = obj.trajDistrib.numJoints - obj.numCtl;
            Kp_t = K_t(:,1:nJoints);            
            Kd_t = K_t(:,(1:nJoints)+nJoints);
            
        end
        
        function [K_t_all, k_t_all, Sigma_u_all] = getCtlGains (obj, mu_x_all, Sigma_t_all )
            
            
            nJoints = obj.trajDistrib.numJoints - obj.numCtl;
            
            idxPos = ((1:obj.trajDistrib.numJoints)-1)*obj.numTimeSteps;
            idxVel = idxPos + obj.numTimeSteps * obj.trajDistrib.numJoints;
            idx_off = [idxPos, idxVel];
            
            idxCtl_off = ((1:obj.numCtl)-1)*obj.numTimeSteps;
            
            idxObs = [1:nJoints, obj.trajDistrib.numJoints + (1:nJoints)];
            idxCtl = nJoints+(1:obj.numCtl);

            sigmaMax = zeros(1, numel(idxObs));
            for t=1:(obj.numTimeSteps)
                
                idx = idx_off + t;
                
                Sigma_t_joint = Sigma_t_all(idx,idx);

                sigmaMax = max([sigmaMax; diag(Sigma_t_joint(idxObs,idxObs))']);
            end
            
            K_t_all = zeros(obj.numTimeSteps*obj.numCtl,2*nJoints);
            k_t_all = zeros(obj.numTimeSteps*obj.numCtl,1);
            Sigma_u_all = zeros(obj.numTimeSteps*obj.numCtl,obj.numCtl);
            
            for t=1:(obj.numTimeSteps)
                
                idx = idx_off + t;
                
                Sigma_t_joint = Sigma_t_all(idx,idx);
                mu_x_joint    = mu_x_all(idx);
                
                % \Psi \Sig_w \Phi^T
                Sig_t_cross = Sigma_t_joint(idxCtl, idxObs);
                                
                % \Phi \Sig_w \Phi^T              
                Sig_t_obs = Sigma_t_joint(idxObs, idxObs);
                Sig_t_obs = Sig_t_obs + diag(sigmaMax) * obj.regularizationFactorModelFreeProMPCtrl;
                
                % \Psi \Sig_w \Psi^T
                Sig_t_ctl = Sigma_t_joint(idxCtl, idxCtl);
                
                K_t =  Sig_t_cross / Sig_t_obs;
                
                k_t1 = mu_x_joint(idxCtl) - K_t * mu_x_joint(idxObs);
                
                Sigma_u = Sig_t_ctl - K_t * Sig_t_cross';
                
                idxCtl_t = idxCtl_off + t;
                K_t_all(idxCtl_t,:) = K_t;
                k_t_all(idxCtl_t,1) = k_t1;
                
                if (obj.estimateNoise)
                    Sigma_uNew = max(0, Sigma_u - obj.SysActionNoise );
                    Sigma_uNew = Sigma_uNew + obj.SysActionNoise;
                    Sigma_u_all(idxCtl_t,:) = diag(diag(Sigma_uNew)); %TODO
                else
                    Sigma_u_all(idxCtl_t,:) = obj.SysActionNoise;
                end

            end
            
        end
        
            
    end

end
