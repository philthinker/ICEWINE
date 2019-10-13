classdef DynamicMovementPrimitives < TrajectoryGenerators.LinearTrajectoryGenerator

    properties(SetObservable,AbortSet)
        
        numSmoothingSteps = 0;
        alphaX = 25;
        betaX = 25/4;
            
    end
    
    properties(SetAccess=protected)
        forcingFunction = [];    
    end
    
    methods (Static)
        
        function [obj] = createFromTrial(trial)

            if (isprop(trial, 'phaseGenerator') && ~isempty(trial.phaseGenerator))
                obj = TrajectoryGenerators.DynamicMovementPrimitives(trial.dataManager, trial.numJoints, trial.phaseGenerator, trial.basisGenerator);            
            else               
                obj = TrajectoryGenerators.DynamicMovementPrimitives(trial.dataManager, trial.numJoints);
            end
        end
        
        
        function [obj] = createDiscreteDMP(dataManager, numJoints)

            phaseGenerator = TrajectoryGenerators.PhaseGenerators.DMPPhaseGenerator(dataManager);              
            basisGenerator = TrajectoryGenerators.BasisFunctions.DMPBasisGenerator(dataManager, phaseGenerator);

            obj = TrajectoryGenerators.DynamicMovementPrimitives(dataManager, numJoints, phaseGenerator, basisGenerator);
            
        end
        
        function [obj] = createRhythmicDMP(dataManager, numJoints)

            phaseGenerator = TrajectoryGenerators.PhaseGenerators.DMPPhaseGeneratorRhythmic(dataManager);              
            basisGenerator = TrajectoryGenerators.BasisFunctions.DMPBasisGeneratorRhythmic(dataManager, phaseGenerator);

            obj = TrajectoryGenerators.DynamicMovementPrimitives(dataManager, numJoints, phaseGenerator, basisGenerator);
            
        end
        
    end
    
    methods
        %%
        function obj = DynamicMovementPrimitives(dataManager, numJoints, phaseGenerator, basisGenerator)
                      
            if (~exist('phaseGenerator', 'var') || isempty(phaseGenerator))
                phaseGenerator = TrajectoryGenerators.PhaseGenerators.DMPPhaseGenerator(dataManager);
            end

            if (~exist('basisGenerator', 'var') || isempty(basisGenerator))
                basisGenerator = TrajectoryGenerators.BasisFunctions.DMPBasisGenerator(dataManager, phaseGenerator);
            end
            
            obj = obj@TrajectoryGenerators.LinearTrajectoryGenerator(dataManager, numJoints, phaseGenerator, basisGenerator, 1000);                        
            
            % trajectory parameters
            obj.linkProperty('numSmoothingSteps');
            obj.linkProperty('alphaX');
            obj.linkProperty('betaX');
            
            level = dataManager.getDataManagerDepth('steps') - 1;
            
            obj.registerOptionalParameter('StartPos', false, obj.numJoints, -1, 1, 'contexts', level);
            obj.registerOptionalParameter('StartVel', false, obj.numJoints, -1, 1, 'contexts', level);
 
            obj.registerOptionalParameter('GoalPos', false, obj.numJoints, -pi, pi, 'parameters');
            obj.registerOptionalParameter('GoalVel', false, obj.numJoints, -10, 10, 'parameters');
            %obj.registerOptionalParameter('Tau', 1, 0.8, 1.2, 'parameters');
            obj.registerOptionalParameter('AmplitudeModifier', false, obj.numJoints, -1, 1, 'parameters');
            
            obj.setIfNotEmpty('StartPos', zeros(1,obj.numJoints) );
            obj.setIfNotEmpty('StartVel', zeros(1,obj.numJoints) );
            obj.setIfNotEmpty('GoalPos' , zeros(1,obj.numJoints) );
            obj.setIfNotEmpty('GoalVel' , zeros(1,obj.numJoints) );
            obj.setIfNotEmpty('AmplitudeModifier', ones(1,obj.numJoints) );
                       
            obj.registerTrajectoryFunction();
        end
        
        
        function [tau] = getTau(obj)
            tau = obj.phaseGenerator.Tau;
        end
        
        %% Trajectory Generation
        function [referencePos, referenceVel, referenceAcc] = getReferenceTrajectory(obj, basis, varargin)
            obj.inputParameterDeMux(varargin);  
            
            if ( obj.useWeights || obj.phaseGenerator.useTau || isempty( obj.forcingFunction )  )
                w = reshape ( obj.Weights, obj.numBasis, [] );
                obj.forcingFunction = basis * w;
            end
            goalPos = [obj.GoalPos, zeros(1, obj.numJoints - numel(obj.GoalPos))];
            goalVel = [obj.GoalVel, zeros(1, obj.numJoints - numel(obj.GoalVel))];
            
            startPos = [obj.StartPos, zeros(1, obj.numJoints - numel(obj.StartPos))];
            startVel = [obj.StartVel, zeros(1, obj.numJoints - numel(obj.StartVel))];
            
            AmplitudeModifier = [obj.AmplitudeModifier, zeros(1, obj.numJoints - numel(obj.AmplitudeModifier))];
            
            [referencePos, referenceVel] = ...                
                            TrajectoryGenerators.DynamicMovementPrimitivesMex( startPos, startVel, goalPos, goalVel, ...
                                                                               obj.alphaX, obj.betaX,  obj.forcingFunction' , ...
                                                                               AmplitudeModifier, obj.getTau(), obj.dt,  ...
                                                                               obj.numTimeSteps + obj.numSmoothingSteps );

            referencePos = referencePos';
            referenceVel = referenceVel';
            referenceAcc = [diff(referenceVel) / obj.dt; zeros(1, obj.numJoints)];
                       
        end        
       
                        
    end
end

