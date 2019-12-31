classdef ParameterDistributionImitationLearner < Learner.Learner & Learner.AbstractInputOutputLearnerInterface
    
    properties
        imitationLearnerTrajectory
        distributionLearner
        trajectoryGenerator
        dataManager         
    end
    
    properties (AbortSet, SetObservable)
        addInitialSigmaToImitation = false;
    end
       
    methods (Static)

        function obj = createFromTrial(trial)
            
            imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(trial.dataManager, trial.trajectoryGenerator, 'jointPositions');
            distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(trial.dataManager, trial.trajectoryGenerator.distributionW);

            obj = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (trial.dataManager, imitationLearner, distributionLearner, trial.trajectoryGenerator);
        end

    end
    
    methods
        
        function obj = ParameterDistributionImitationLearner(dataManager, imitationLearnerTrajectory, distributionLearner, trajectoryGenerator)
            obj = obj@Learner.Learner();
            obj = obj@Learner.AbstractInputOutputLearnerInterface(dataManager, distributionLearner.functionApproximator);
            
            obj.dataManager = dataManager;
            obj.trajectoryGenerator = trajectoryGenerator;
            obj.imitationLearnerTrajectory = imitationLearnerTrajectory;            
            obj.distributionLearner = distributionLearner;
            obj.linkProperty('addInitialSigmaToImitation');
            
            parameterNames = dataManager.getAliasStructure('parameters').entryNames;
            trajectoryParameters = obj.trajectoryGenerator.getParameterNamesForTrajectoryGenerator();
            
            for i = 1:length(parameterNames)
                if (any(strcmp(trajectoryParameters, parameterNames{i})))
                    dataManager.addDataEntry([trajectoryParameters{i}, 'Imitation'], dataManager.getNumDimensions(trajectoryParameters{i}));
                    dataManager.addDataAlias('parametersImitation', [trajectoryParameters{i}, 'Imitation']);                
                else
                    dataManager.addDataAlias('parametersImitation', trajectoryParameters{i});                
                end
            end
            
            obj.distributionLearner.setOutputVariableForLearner([obj.distributionLearner.outputVariable, 'Imitation']);           
        end
        
        function [] = setAddInitialSigma(obj, addInitSigma)
            obj.addInitialSigmaToImitation = addInitSigma;
        end
        
        function [] = setWeightName(obj, weightName)
            obj.setWeightName@Learner.AbstractInputOutputLearnerInterface(weightName);
            obj.distributionLearner.setWeightName(weightName);
        end    
                        
        function obj = updateModel(obj, data)
            % Save initial variance (if we are in the first update, i.e.,
            % only imitation learning
            if (obj.addInitialSigmaToImitation && obj.iteration == 0)
                SigmaInitial = obj.distributionLearner.functionApproximator.getCovariance();                                
            end
            
            % Compute the parameters from imitation data (do not take the parameters directly 
            % from the data set
            obj.trajectoryGenerator.disableParametersFromData();
            for i = 1:data.getNumElementsForDepth(1) %num of trajectories 
                obj.imitationLearnerTrajectory.learnTrajectory(data, i);  
                obj.trajectoryGenerator.registerAdditionalParametersInData(data, 'Imitation', i);                               
            end
            obj.trajectoryGenerator.enableParametersFromData();
            
            %Now learn the distribution (initial distribution from imitation)
            obj.distributionLearner.updateModel(data);
            
            % For computing the getReferenceTrajectory with the mean
            % obj.trajectoryGenerator.Weights = obj.distributionLearner.functionApproximator.getMean();
            
            % Add the initial variance to the initial distribution (this is
            % for example useful if we only have one datapoint)
            if (obj.addInitialSigmaToImitation && obj.iteration == 0)
                SigmaANew = obj.distributionLearner.functionApproximator.getCovariance();
                SigmaANew = SigmaANew + SigmaInitial;
                obj.distributionLearner.functionApproximator.setCovariance(SigmaANew);
            end
        end
    end
    
end