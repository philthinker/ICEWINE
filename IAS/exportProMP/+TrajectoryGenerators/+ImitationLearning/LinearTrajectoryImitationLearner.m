classdef LinearTrajectoryImitationLearner < Learner.Learner
    
    properties(AbortSet, SetObservable)
        imitationLearningRegularization = 10^-6;
        useWeights;
        
        useJerkPenalty=false;
    end
    
    properties
        trajectoryGenerator
        phaseGenerator
        
        setFixedParameters
        trajectoryName
    end
    
    methods
        
        function obj = LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, trajectoryName, setFixedParameters)
            obj = obj@Learner.Learner();
            obj.linkProperty('imitationLearningRegularization');
            obj.linkProperty('useWeights');
            obj.linkProperty('useJerkPenalty');
            
            obj.trajectoryGenerator = trajectoryGenerator;
            obj.phaseGenerator = trajectoryGenerator.phaseGenerator;            
            
            if (exist('trajectoryName', 'var'))
                obj.trajectoryName = trajectoryName;
            else
                obj.trajectoryName = 'jointPositions';
            end
            
            if (~exist('setFixedParameters', 'var'))
                setFixedParameters = true;
            end
            obj.setFixedParameters = setFixedParameters;
        end
        
        function [basisMDOF] = getBasisFunctionsMultiDOF(obj, basis)
            numJoints = obj.trajectoryGenerator.numJoints;
            basisMDOF = zeros(size(basis) * numJoints);
            
            for i = 1:numJoints
                basisMDOF((1:size(basis,1)) + (i-1) * size(basis,1), (1:size(basis,2)) + (i-1) * size(basis,2)) = basis;
            end
        end
        
        function [targetFunction] = getTargetFunctionForImitation(obj, q, qd, qdd)
            targetFunction = q(:);
        end
        
        function [] = setMetaParametersFromTrajectory(obj, q, qd, qdd)
            obj.phaseGenerator.setMetaParametersFromTrajectory(q);
        end
        
        function [Yd, Ydd] = getDiffVelocitiesAndAccelerations(obj, Y)
            Yd  = (Y(2:end,:) - Y(1:end-1,:)) / obj.trajectoryGenerator.dt;
            Yd = ([Yd; Yd(end,:) ]);
            
            Ydd = (Yd(2:end,:) - Yd(1:end-1,:)) / obj.trajectoryGenerator.dt;
            Ydd = ([Ydd; Ydd(end,:) ]);
        end
        
        function [] = learnWeights(obj, basis, targetTrajectory, targetTrajectoryD, targetTrajectoryDD)
            
            targetFunction = obj.getTargetFunctionForImitation(targetTrajectory, targetTrajectoryD, targetTrajectoryDD);
            
            nStepsOver = 10;
            newDt = obj.trajectoryGenerator.phaseGenerator.dt / 5;
            phaseStart =  - nStepsOver * newDt;
            phaseEnd   = obj.trajectoryGenerator.phaseGenerator.phaseEndTime + nStepsOver * newDt;
            
            x = obj.trajectoryGenerator.phaseGenerator.generatePhaseFromTime(phaseStart:newDt:phaseEnd);
          
            basisMDOF = obj.getBasisFunctionsMultiDOF(basis);
            
            if ( obj.useJerkPenalty )
                %             basisDD = obj.trajectoryGenerator.basisGenerator.generateBasisDD(x');
                basisDDD = obj.trajectoryGenerator.basisGenerator.generateBasisDDD(x');
                basisDDMDOF = obj.getBasisFunctionsMultiDOF(basisDDD);
                %             reg = diag(obj.imitationLearningRegularization ./ sum(basisMDOF));
                reg = obj.imitationLearningRegularization * (basisDDMDOF'*basisDDMDOF);
            else
                reg = obj.imitationLearningRegularization * eye(size(basisMDOF' * basisMDOF));
            end
            
            Weights = (basisMDOF' * basisMDOF + reg ) \ basisMDOF' * targetFunction;
            obj.trajectoryGenerator.Weights = Weights;            
        end
        
        function [] = learnTrajectory(obj, data, index)
            targetTrajectory = data.getDataEntry(obj.trajectoryName, index);
            [targetTrajectoryD, targetTrajectoryDD] =  obj.getDiffVelocitiesAndAccelerations(targetTrajectory);
            
            obj.setMetaParametersFromTrajectory(targetTrajectory, targetTrajectoryD, targetTrajectoryDD);
            basis = data.getDataEntry('basis', index);
            
            if (obj.useWeights || obj.setFixedParameters)
                obj.learnWeights(basis, targetTrajectory, targetTrajectoryD, targetTrajectoryDD);
            end                        
        end
        
        function obj = updateModel(obj, data)
            obj.learnTrajectory(data, 1);            
        end
    end
    
end