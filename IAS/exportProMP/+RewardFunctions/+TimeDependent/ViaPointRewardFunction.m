classdef ViaPointRewardFunction < RewardFunctions.TimeDependent.TimeDependentRewardFunction & RewardFunctions.RewardFunctionSeperateStateActionInterface & Data.OptionalParameterInterface
    
    properties(SetObservable, AbortSet)
        viaPointTimes
        viaPointFactors
        viaPoints
        viaPointNoise = 0;
        
        uFactor
    end
    
    properties
        stateFeatures       = 'states';
        nextStateFeatures   = 'nextStates';
        actionName          = 'actions';
    end
    
    
    methods
        function obj = ViaPointRewardFunction(dataManager, viaPointTimes, viaPoints, viaPointFactors, uFactor, ...
                stateFeatures, nextStateFeatures, actionName)
            
            obj = obj@RewardFunctions.TimeDependent.TimeDependentRewardFunction(dataManager);
            obj = obj@RewardFunctions.RewardFunctionSeperateStateActionInterface();    
            
            if (exist('stateFeatures', 'var'))
                obj.stateFeatures = stateFeatures ;
            end

            if (exist('nextStateFeatures', 'var'))
                obj.nextStateFeatures = nextStateFeatures;
            end
            
            if (exist('parameterName', 'var'))
                obj.actionName = actionName ;
            end

            
            obj.viaPointTimes = viaPointTimes;
            obj.viaPoints = viaPoints;
            obj.viaPointFactors = viaPointFactors;
            obj.uFactor = uFactor;
                        
            obj.linkProperty('viaPointTimes');
            obj.linkProperty('viaPointNoise');
            obj.linkProperty('viaPointFactors');
            obj.linkProperty('viaPoints');            
            obj.linkProperty('uFactor');       
                                    
            obj.registerTimeDependentRewardFunctions();
        end        

        function [] = registerTimeDependentRewardFunctions(obj) 
            obj.setRewardInputs(obj.stateFeatures, obj.actionName, 'timeSteps', obj.additionalParameters{:});            
            obj.addDataManipulationFunction('sampleFinalReward', {obj.nextStateFeatures, 'timeSteps', obj.additionalParameters{:}}, {'finalRewards'}, false);  
        end
                        
        function [reward, stateReward, actionReward] = rewardFunction(obj, q, u, timeSteps, varargin)
            stateReward =  obj.getViaPointReward(q, timeSteps, varargin{:});
            actionReward = - sum(bsxfun(@times, u.^2, obj.uFactor),2);
            reward = actionReward + stateReward;            
        end
        
        function [vargout] = finalReward(obj, finalStates, varargin)
            vargout = obj.getViaPointReward(finalStates, obj.numTimeSteps + 1, varargin{:});
        end
        
        function [viaPoint] = getViapoint(obj, i, j, numSamples, varargin)
            viaPoint = obj.viaPoints{i}(j, :);
            viaPoint = bsxfun(@plus, randn(numSamples, size(viaPoint,2)) .* obj.viaPointNoise, viaPoint);
        end
        
        function [viapointReward] = getViaPointReward(obj, input, timeSteps, varargin)
            viapointReward = zeros(size(input,1),1);
            jointThetaPunishment = 0;
            q = input;
            %theta = input(:,5:end);
            %jointPositiveThetaFlag = theta(:,1) > 0;
            %jointPositiveThetaFlag2 = theta(:,2) < 0;
            %viapointReward = viapointReward +(jointThetaPunishment*jointPositiveThetaFlag); 
            
            for i = 1:length(obj.viaPointTimes)
                indices = timeSteps == obj.viaPointTimes(i);
                if (sum(indices) > 0)
                    rewardTmp = -inf(sum(indices),1);
                    for j = 1:size(obj.viaPoints{i},1)

                        rewardTmp = max(rewardTmp, - (q(indices,:) - obj.getViapoint(i, j, size(q,1), varargin{:})).^2*obj.viaPointFactors(i,:)');
                    end
                    viapointReward(indices) = viapointReward(indices) + rewardTmp;
                end
                
            end
        end
        
        %return reward and quadratic approximation
        % ct = - u'H u - 2 u h - q' R q - 2 q r
        function [ct, H, h, R, r] = getQuadraticCosts(obj, q, u, k)
            r = zeros(length(q),  1);
            R = -eye(length(q)) * 10^-6;
            
            h = zeros(1, length(u));
            H = - eye(length(u)) * obj.uFactor;
            
            ct =  - u * H * u' - 2 * h * u';
            timeIndex = find(obj.viaPointTimes == k);
            
            if (isempty(timeIndex))
                return
            end
            
            if (~iscell(obj.viaPointFactors))
                
                reward = -inf;
                rewardIndex = 1;
                for j = 1:size(obj.viaPoints{timeIndex},1)
                    rewardTmp = - sum(obj.viaPointFactors(timeIndex,:) .* (q - obj.viaPoints{timeIndex}(j, :)).^2);
                    if (rewardTmp > reward)
                        reward = rewardTmp;
                        rewardIndex = j;
                    end
                    R = - diag(obj.viaPointFactors(timeIndex,:));
                    r = R * obj.viaPoints{timeIndex}(rewardIndex, :)';
                end
                
            else
                reward = -inf;
                rewardIndex = 1;
                
                for j = 1:size(obj.viaPoints{timeIndex},1)
                    rewardTmp = - (q - obj.viaPoints{timeIndex}(j, :)) * obj.viaPointFactors{timeIndex,j} * (q - obj.viaPoints{timeIndex}(j, :))';
                    if (rewardTmp > reward)
                        reward = rewardTmp;
                        rewardIndex = j;
                    end
                end
                R = - obj.viaPointFactors{timeIndex,rewardIndex};
                r = R * obj.viaPoints{timeIndex}(rewardIndex, :)';
            end
            ct = ct -  q * R * q' - 2 * q * r;
        end
    end
end
