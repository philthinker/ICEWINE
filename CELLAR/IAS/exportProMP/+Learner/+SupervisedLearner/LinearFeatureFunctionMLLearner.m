classdef LinearFeatureFunctionMLLearner < Learner.SupervisedLearner.SupervisedLearner & Learner.ParameterOptimization.HyperParameterObject
    % The LinearFeatureFunctionMLLearner is a Learner.SupervisedLearner.SupervisedLearner
    % that will model a learner via weighted linear regression
    %
    % The function to learn will be modeled by a <tt>Function.FunctionLinearInFeatures</tt> 
    % which has to be given to the constructor.  The parameters of this function will be 
    % estimated by calling <tt>learnFunction</tt> via weighted linear regression using a ridge term.
    %
    % The flags <tt>inputDataNormalization</tt> and <tt>outputDataNormalization</tt> 
    % determine if the the function <tt>learnFunction()</tt> will normalize 
    % the input and output data to <tt>sigma = 1 </tt> and <tt>mean = 0</tt> respectively. 
    properties(SetObservable, AbortSet)
        inputDataNormalization = true; % learnFunction() will normalize the input if set
        outputDataNormalization = true; % learnFunction() will normalize the output if set
        
        regularizationRegression = 10^-6;
    end
    
    % Class methods
    methods
        function obj = LinearFeatureFunctionMLLearner(dataManager, linearfunctionApproximator, varargin)
            % @param dataManager Data.DataManger to operate on
            % @param linearfunctionApproximator function object that will be learned 
            % @param varargin contains the following optional arguments in this order: weightName, inputVariables, outputVariable (see superclass SupervisedLearner)
            obj = obj@Learner.SupervisedLearner.SupervisedLearner(dataManager, linearfunctionApproximator, varargin{:});
            obj.linkProperty('regularizationRegression');
        end
        
        function [] = learnFunction(obj, inputData, outputData, weighting)
            % @param inputData input data of the training set
            % @param outputData output data of the training set
            % @param weighting optional weighting for each datapoint 
            % This function will approximate the function via weighted linear
            % regression. If the weighting is not given a default weight of 1 for all data points is used. 
            %
            % This function does not remember the data from previous calls so be 
            % sure to always use your complete dataset.
            if (~exist('weighting', 'var'))
                % If no weights are specified default all to 1
                weighting = ones(size(outputData,1),1);
            end
            numSamples   = size(outputData,1);
            
            if(obj.outputDataNormalization)
                % if outputDataNormalization is set, transform the output mean to 0 and output range to [-1,1]
                rangeOutput = obj.dataManager.getRange(obj.functionApproximator.outputVariable);
                meanRangeOutput = (obj.dataManager.getMinRange(obj.functionApproximator.outputVariable) + obj.dataManager.getMaxRange(obj.functionApproximator.outputVariable)) / 2;
                
                outputData = bsxfun(@rdivide,bsxfun(@minus, outputData, meanRangeOutput), rangeOutput);
            end
            
            if(~isempty(inputData))
                if(obj.inputDataNormalization)
                    rangeInput = std(inputData,[], 1);
                    rangeInput(rangeInput < 1e-4) = 1e-4;
                    meanRangeInput = mean(inputData, 1);
                    
                    inputData = bsxfun(@rdivide, bsxfun(@minus, inputData, meanRangeInput), rangeInput);
                end
            end
            
            valididxs = var(inputData) ~= 0;
            
            if (~isempty(inputData))
                Shat = [ones(numSamples,1), inputData(:, valididxs)];
            else
                valididxs = false(numSamples,1);
                Shat = ones(numSamples,1);
            end
            
            sumW    = sum(weighting); % Sum of the weights
            weighting = weighting / sumW;
            dimInput = size(Shat,2)-1;
            
            SW                  = bsxfun(@times, Shat, weighting);
            thetaL              = (SW'*Shat + obj.regularizationRegression * diag([0; ones(dimInput,1)]) ) \ SW' * outputData ;
            
            thetaL              = thetaL';
            
            MuA                 = thetaL(:,1);
            BetaA               = zeros(size(outputData,2),size(inputData,2));
            BetaA(:,valididxs)  = thetaL(:,2:end);
            
            if(obj.inputDataNormalization && ~isempty(inputData))
                BetaA = bsxfun(@rdivide, BetaA, rangeInput);
                MuA = MuA - BetaA * meanRangeInput';
            end
            
            if(obj.outputDataNormalization)
                MuA = MuA .* rangeOutput' + meanRangeOutput';
                BetaA = BetaA .* repmat(rangeOutput', 1, size(BetaA,2));
            end
            
            obj.functionApproximator.setWeightsAndBias(BetaA, MuA);
            
        end

        function params = getHyperParameters(obj)
            if(ismethod(obj.functionApproximator, 'getHyperParameters'))
                params = [obj.regularizationRegression, obj.functionApproximator.getHyperParameters ];
            else
                params = obj.regularizationRegression;
            end
        end
        
        function setHyperParameters(obj,params)
            obj.regularizationRegression = params(1);
            if(ismethod(obj.functionApproximator, 'getHyperParameters'))
                obj.functionApproximator.setHyperParameters(params(2:end));
            end
            
        end
        
        function num = getNumHyperParameters(obj)
            if(ismethod(obj.functionApproximator, 'getHyperParameters'))
                num = 1 + obj.functionApproximator.getNumHyperParameters;
            else
                num = 1;
            end            
        end
        
        function [expParameterTransformMap] = getExpParameterTransformMap(obj)
            if(ismethod(obj.functionApproximator, 'getExpParameterTransformMap'))
                expParameterTransformMap = [obj.functionApproximator.getExpParameterTransformMap(), true];
            else
                expParameterTransformMap = true;
            end 
        end

    end
    
end
