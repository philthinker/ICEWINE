classdef TimeVarLinearControllerDistributionCorrection < TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController
    
    properties(AbortSet, SetObservable)
        PGains
        DGains
        
        correctionThreshold = 10;
        correctionTemperature = 1;
    end
    
    properties
        distribution
        
        mu_t
        Sigma_t
        
    end
    
    methods
        
        function obj = TimeVarLinearControllerDistributionCorrection(dataManager, distribution, numJoints, gainProvider)
            obj = obj@TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainProvider); 
            
            obj.PGains = ones(1, numJoints) * 100;
            obj.DGains = ones(1, numJoints) * 20;
            
            obj.distribution = distribution;
            
            obj.linkProperty('PGains', 'PGainsDistributionCorrection');
            obj.linkProperty('DGains', 'DGainsDistributionCorrection');
            
            obj.linkProperty('correctionTemperature');
            obj.linkProperty('correctionThreshold');
            
            
            obj.addDataManipulationFunction('updateModel', {}, ...
                      {}, Data.DataFunctionType.ALL_AT_ONCE, false );                                                        
            obj.setTakesData('updateModel', true);


        end
        
        function [] = registerTrackingFunction(obj)
            obj.dataManager.getSubDataManager.addDataEntry('activationPD', 1);
            obj.addDataManipulationFunction('getTrackingControl', {'jointPositions', 'jointVelocities','timeSteps'}, {'actions', 'activationPD'});
            obj.addDataFunctionAlias('sampleAction','getTrackingControl');
        end 
        
        function [] = updateModel(obj, data)
            
            [~, temp2] = data.getNumElementsForDepth(2);
            numTimeSteps = temp2(1);
            
            for i  = 1:numTimeSteps 
                [obj.mu_t(i, :), obj.Sigma_t(i,:, :) ] = obj.distribution.callDataFunctionOutput('getExpectationAndSigma',data, 1, i);
            end
            
        end
        
        function [action, softActivation] = getTrackingControl(obj, jointPos, jointVel, timeSteps)
            [ Kp, Kd, kff, actNoise ] = obj.gainProvider.getFeedbackGainsForT(timeSteps);
            referenceTrajectory = obj.mu_t(timeSteps, 1:obj.numJoints);
            referenceTrajectoryD = obj.mu_t(timeSteps, (1:obj.numJoints) + obj.numJoints * 2);

            actionFF = bsxfun(@plus, [ Kp, Kd ] * [ referenceTrajectory'; referenceTrajectoryD' ], kff )';

            actionDist = bsxfun(@plus, [ Kp, Kd ] * [ jointPos'; jointVel' ], kff )' ...
                   + bsxfun(@times, randn(length(timeSteps),obj.numJoints), sqrt(diag(actNoise))');
               
            
            action = bsxfun(@times, (referenceTrajectory - jointPos), obj.PGains) + bsxfun(@times, (referenceTrajectoryD - jointVel), obj.DGains);
            action = bsxfun(@plus, action, actionFF);
            
            index = [1:obj.numJoints, (1:obj.numJoints) + obj.numJoints * 2];
            mu_state = obj.mu_t(timeSteps, index);
            Sigma_state = obj.Sigma_t(timeSteps(1), index, index);
            
            cholSigma = chol(squeeze(Sigma_state));
            tempDiff = (mu_state - [jointPos, jointVel]) / cholSigma;
            log_like = -log(2*pi) * size(cholSigma,1) - sum(log(eig(cholSigma))) - 0.5 * sum(tempDiff.^2, 2);
            
            
            softActivation = 1 ./ (1 + exp(- (log_like + obj.correctionThreshold * size(mu_state,2)) / obj.correctionTemperature));
           
            action = bsxfun(@times, action, (1-softActivation)) + bsxfun(@times, actionDist, softActivation);
        end
        
        
    end
    
    
end