classdef LinearizedPlanner < TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider & Learner.Learner & Data.DataManipulator
    
    properties         
        
        trajectoryUpdater;
        
        linearModelProvider;
        quadraticRewardProvider;
        
        % This defines our controller, we need to set it in the doPlanning
        % method
        
        controllerArray
        stateDistributionArray
        actionDistributionArray
        valueFunctionArray
        
        linearModel
        quadraticReward
        
        numStates;
        numActions;
    end
    
    properties (AbortSet, SetObservable)
        numTimeSteps;
        numPlanningIterations = 1;
                
        initSigmaActions = 1.0;
    end
    
    methods
        function obj = LinearizedPlanner(dataManager, trajectoryUpdater, linearModelProvider, quadraticRewardProvider)
            obj = obj@TrajectoryGenerators.TrajectoryTracker.IFbGainsProvider();
            obj = obj@Learner.Learner();
            obj = obj@Data.DataManipulator(dataManager);
            
            obj.numStates=obj.dataManager.getNumDimensions('states');
            obj.numActions=obj.dataManager.getNumDimensions('actions');
            
            obj.trajectoryUpdater = trajectoryUpdater;
            obj.linearModelProvider = linearModelProvider;
            obj.quadraticRewardProvider = quadraticRewardProvider;
            
            obj.linkProperty('numTimeSteps');
            obj.linkProperty('numPlanningIterations');
            obj.linkProperty('initSigmaActions');
                                    
        end
        
        function [] = updateModel(obj, data)
            initialStates = data.getDataEntry('states',:, 1);
                        
            obj.doPlanning(1, mean(initialStates), cov(initialStates));
        end
        
        function [controllerArray] = createControllerArray(obj, numTimeSteps)
            controllerStruct.K = zeros(obj.numActions, obj.numStates);
            controllerStruct.kff = zeros(obj.numActions, 1);
            controllerStruct.SigmaCtl = eye(obj.numActions) * obj.initSigmaActions;
            controllerArray(1:numTimeSteps) = controllerStruct;            
        end
        
        function [valueArray] = createValueFunctionArray(obj, numTimeSteps)
            valueStruct.V = zeros(obj.numStates, obj.numStates);
            valueStruct.v = zeros(obj.numStates, 1);
            valueStruct.v0 = 0;
            valueStruct.v0softMax = 0;
            
            valueArray(1:numTimeSteps) = valueStruct;            
        end
        
        function [stateDistArray] = createStateDistributionArray(obj, numTimeSteps)
            stateDist.Sigma = zeros(obj.numStates, obj.numStates);
            stateDist.Mu = zeros(1,obj.numStates);
            
            stateDistArray(1:numTimeSteps) = stateDist;            
        end                
        
                        
        function [ K_t, kff_t, SigmaCtl_t ] = getFeedbackGainsForT(obj, ~, timesteps, ~)
            
            t = timesteps(1);
            K_t = obj.controllerArray(t).K;
            kff_t = obj.controllerArray(t).kff;
            SigmaCtl_t = obj.controllerArray(t).SigmaCtl;
        end
                

        function [] = doPlanning(obj, startTimestep, startMu, startSigma)
            stopPlanning = false;
            iteration = 1;
            
            [obj.controllerArray] = obj.createControllerArray(obj.numTimeSteps);
            [obj.valueFunctionArray] = obj.createValueFunctionArray(obj.numTimeSteps);
            [obj.stateDistributionArray] = obj.createStateDistributionArray(obj.numTimeSteps+1);
                        
            while (~stopPlanning)
                obj.updateLinearizationModels(iteration, startTimestep, startMu, startSigma);
                
                obj.doPlanningInnerLoop(startTimestep, startMu, startSigma);
                
                stopPlanning = obj.stoppingCriterionOuterLoop(iteration);
                iteration = iteration + 1;
            end
        end
        
        function [linearModels] = getLinearModels(obj)
            linearModels = obj.linearModel;
        end
        
        function [quadraticReward] = getQuadraticRewards(obj)
            quadraticReward = obj.quadraticReward;
        end
        
        function [controllers] = getController(obj)
            controllers = obj.controllerArray;
        end
        
        function [] = updateLinearizationModels(obj, iteration, startTimestep, startMu, startSigma)
            % We update our trajectory (standard behaviour: just
            % simulate the feedback controller without noise (in
            % controller and environment)
            obj.trajectoryUpdater.updateTrajectories(iteration, obj, startTimestep, startMu, startSigma);
            
            % the linear model and quadratic reward use the new
            % trajectory to get a new model
           obj.linearModel = obj.linearModelProvider.getLinearizedModel(obj.trajectoryUpdater);
           obj.quadraticReward = obj.quadraticRewardProvider.getQuadraticRewardModel(obj.trajectoryUpdater);
        end
        
        
       
        
        function [] = doPlanningInnerLoop(obj, startTimestep, startMu, startSigma)
            innerLoop = true;
            iteration = 1;
            controllerArray = obj.controllerArray;
            while (innerLoop)
                
                % do forward pass via belief propagation (updates obj.stateDistributionArray)                
                obj.forwardPass(startTimestep, startMu, startSigma, obj.getLinearModels(), obj.getController());
                
                % Here we can set algorithmic specific additional reward
                % terms
                internalRewardArray = obj.setInternalRewardModel(obj.stateDistributionArray, obj.getQuadraticRewards());
                
                % now do the backward pass to obtain value function and new
                % controller (updates obj.valueFunctionArray and
                % obj.controllerArray)
                obj.backwardPass(startTimestep, internalRewardArray, obj.getLinearModels());                
                
                obj.updateDualVariables(obj.stateDistributionArray, obj.valueFunctionArray, obj.controllerArray);
                innerLoop = ~obj.stoppingCriterionInnerLoop(iteration, obj.stateDistributionArray, obj.valueFunctionArray, obj.controllerArray);
                iteration = iteration + 1;
            end
        end
        
        function [] = forwardPass(obj, startTimeStep, startMu, startSigma, linearModel, controllerArray)
            obj.stateDistributionArray(startTimeStep).Mu = startMu;
            obj.stateDistributionArray(startTimeStep).Sigma = startSigma;
            obj.actionDistributionArray(startTimeStep).Sigma = [];
            obj.actionDistributionArray(startTimeStep).Mu = [];

            for i  = startTimeStep:obj.numTimeSteps
                [obj.stateDistributionArray(i  + 1), obj.actionDistributionArray(i)] = obj.forwardPassSingleStep(obj.stateDistributionArray(i), linearModel(i), controllerArray(i));    
            end
        end
        
        function [internalRewardArray] = setInternalRewardModel(obj, stateDistributionArray, rewardFunctionArray)
            internalRewardArray = rewardFunctionArray;
        end               
        
        function [] = updateDualVariables(obj, stateDistributionArray, valueFunctionArray, controllerArray)
            % update the dual variables (eta and so on...)
        end
        
        function [nextStateDistribution, currentActionDistribution] = forwardPassSingleStep(obj, currentState, currentModel, currentController)
            currentActionDistribution.Sigma=currentController.SigmaCtl+currentController.K*currentState.Sigma*currentController.K';
            currentActionDistribution.Mu=currentController.K*currentState.Mu'+currentController.kff;
            currentStateActionDistribution.Mu=[currentState.Mu'; currentActionDistribution.Mu];
            currentStateActionDistribution.Sigma=[currentState.Sigma, currentState.Sigma*currentController.K';...
                                                  currentController.K*currentState.Sigma, currentActionDistribution.Sigma];

            % Attention: The structure has to be build exactly in this order...
            nextStateDistribution.Sigma = currentModel.H + [currentModel.A, currentModel.B] * currentStateActionDistribution.Sigma * [currentModel.A'; currentModel.B'];         
            nextStateDistribution.Sigma = 0.5*(nextStateDistribution.Sigma+nextStateDistribution.Sigma');
            nextStateDistribution.Mu = currentStateActionDistribution.Mu'*[currentModel.A, currentModel.B]' + currentModel.c';
        end
        
        function [] = backwardPass(obj, startTimeStep, internalReward, linearModel)
            %obj.numTimeSteps=obj.numTimeSteps-1;
            obj.valueFunctionArray(obj.numTimeSteps + 1).V = internalReward(obj.numTimeSteps + 1).Rss;
            obj.valueFunctionArray(obj.numTimeSteps + 1).v = internalReward(obj.numTimeSteps + 1).rs;
            obj.valueFunctionArray(obj.numTimeSteps + 1).v0 = internalReward(obj.numTimeSteps + 1).r0;
            obj.valueFunctionArray(obj.numTimeSteps + 1).v0softMax = obj.valueFunctionArray(obj.numTimeSteps + 1).v0;
 
            for i = obj.numTimeSteps: - 1 : startTimeStep
                [obj.controllerArray(i), obj.valueFunctionArray(i)] = obj.backwardPassSingleStep(internalReward(i), linearModel(i), obj.valueFunctionArray(i + 1));
            end
        end
        
        function [controllerStruct, valueFunctionStruct] = backwardPassSingleStep(obj, currentReward, currentModel, nextValue)          
            % The following temporary variables correspond to the
            % coefficients of the Q-function: 
            % [s;a]'*[Qss, Qsa; Qas, Qaa]*[s;a] + [s;a]'*[Qs;Qa] + Q0
            Qaa=currentReward.Raa+currentModel.B'*nextValue.V*currentModel.B;
            Qss=currentReward.Rss+currentModel.A'*nextValue.V*currentModel.A;
            Qas=(currentReward.Rsa+currentModel.A'*nextValue.V*currentModel.B)';
            Qa=currentReward.ra' + 2*currentModel.B'*nextValue.V*currentModel.c + currentModel.B'*nextValue.v;
            Qs=currentReward.rs + 2*currentModel.A'*nextValue.V*currentModel.c + currentModel.A'*nextValue.v;
            Q0common=currentReward.r0 + currentModel.c'*nextValue.V*currentModel.c+trace(nextValue.V*currentModel.H)+nextValue.v'*currentModel.c;
            Q0=Q0common+nextValue.v0;
            Q0softMax=Q0common+nextValue.v0softMax;

            % Attention: The structure has to be build exactly in this order...
            controllerStruct.K = -(Qaa\Qas);
            controllerStruct.kff = -0.5*(Qaa\Qa);
            controllerStruct.SigmaCtl = -0.5*inv(Qaa);
            controllerStruct.SigmaCtl = 0.5*(controllerStruct.SigmaCtl'+controllerStruct.SigmaCtl);
            
            valueFunctionStruct.V = Qss+Qas'*controllerStruct.K;
            valueFunctionStruct.v = Qs+2*Qas'*controllerStruct.kff;
            valueFunctionStruct.v0 = 1/2*Qa'*controllerStruct.kff + Q0 - 0.5*obj.numActions;
            valueFunctionStruct.v0softMax = 1/2*Qa'*controllerStruct.kff + Q0softMax + 0.5*(obj.numActions*log(2*pi)-log(det(-2*Qaa)));

            % During the backward pass V might become more and more
            % asymmetric for numerical reasons. We prevent this by
            % restoring symmetry after each time step
            valueFunctionStruct.V = 1/2*(valueFunctionStruct.V+valueFunctionStruct.V');
        end
        
        function [stopInnerLoop] = stoppingCriterionInnerLoop(obj, iterationNumber, stateDistributionArray, valueFunctionArray, controllerArray)
            stopInnerLoop = true;
        end
        
        function [stopPlanning] = stoppingCriterionOuterLoop(obj, iteration)
             % naive stopping criterion (maxIteration)
             
             stopPlanning = iteration >= obj.numPlanningIterations;
        end
    end           
    
end