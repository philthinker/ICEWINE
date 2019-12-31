classdef TimeVarLinearController < TrajectoryGenerators.TrajectoryTracker.AbstractTrajectoryTracker & Distributions.DistributionWithMeanAndVariance & Functions.Mapping
    
    properties(AbortSet, SetObservable)
        linearFeedbackNoiseRegularization = 0;
        callPerSample = false;
        fullNoiseMatrix;        
    end
    
    properties
        gainProvider;
    end
    
    methods
        
        function obj = TimeVarLinearController(dataManager, numJoints, gainProvider)
            obj = obj@TrajectoryGenerators.TrajectoryTracker.AbstractTrajectoryTracker(dataManager, numJoints); %
            obj = obj@Distributions.DistributionWithMeanAndVariance();
            obj = obj@Functions.Mapping(dataManager, 'actions', {{'jointPositions', 'jointVelocities','timeSteps'}}, 'trajectoryTracker'); 
            
            obj.linkProperty('callPerSample');
            obj.linkProperty('fullNoiseMatrix');
            
            obj.gainProvider = gainProvider;
            obj.registerTrackingFunction();
            obj.registerMappingInterfaceDistribution();
            obj.restrictToRangeLogLik = true;
            
            obj.linkProperty('linearFeedbackNoiseRegularization');

        end
        
        function [] = registerTrackingFunction(obj)
            if ( obj.callPerSample ) 
                obj.addDataManipulationFunction('getTrackingControl', {'states','timeSteps'}, {'actions'},Data.DataFunctionType.SINGLE_SAMPLE);
            else
                obj.addDataManipulationFunction('getTrackingControl', {'states','timeSteps'}, {'actions'});
            end
            obj.setTakesData('getTrackingControl', true);
            obj.addDataFunctionAlias('sampleAction','getTrackingControl');
            
        end 
        
        
        function [action] = getTrackingControl(obj, data, states, timesteps)
            [ K, kff, actNoise ] = obj.gainProvider.getFeedbackGainsForT(data, timesteps, states);
            if ( obj.fullNoiseMatrix ) 
                noise = mvnrnd( zeros(1, obj.numJoints), actNoise, length(timesteps) );
            else
                noise = bsxfun(@times, randn(length(timesteps),obj.numJoints), sqrt(diag(actNoise))'); 
            end
            action = bsxfun(@plus, K * [ states'], kff )' + noise;
        end
        
        function [mu, sigma] = getExpectationAndSigma(obj, numElements, input)
            timesteps = input(:, end);
            [timeStepsUnique, idxA, idxC] = unique(timesteps);
            
            mu = zeros(numElements, obj.dimOutput);
            sigma = zeros(numElements, obj.dimOutput);
            for i = 1:length(timeStepsUnique)
                [ K, kff, actNoise ] = obj.gainProvider.getFeedbackGainsForT(timeStepsUnique(i));
                indexTimeStep = timesteps == timeStepsUnique(i);
                states_local = input(indexTimeStep, 1:end-1);                
                
                mu(indexTimeStep, :) = bsxfun(@plus, [ K ] * [ states_local]', kff )';
                sigma(indexTimeStep, :) = repmat(sqrt(diag(actNoise))', sum(indexTimeStep), 1);
            end
            sigma = sigma + obj.linearFeedbackNoiseRegularization;
        end
        
    end
    
    
end