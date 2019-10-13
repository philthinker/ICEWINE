classdef FunctionLinearInFeatures < Functions.Mapping & Functions.Function & ParametricModels.ParametricFunction &  Learner.ParameterOptimization.HyperParameterObject
    % The FunctionLinearInFeatures is a subclass of mapping and implements
    % the Function interface. It models a multidimensional linear function.
    %
    % Since this class is a subclass of mapping you need to define a
    % datamananger to operate on as well as a set of output and input
    % variables and a function Name (See Functions.Mapping for more
    % information about mappings).
    %
    % This class will save the linear function it represents in the variables
    % weights and bias, where weights is the coefficient matrix of the
    % function and bias its offset. As Equation:
    % \f[
    %	\boldsymbol{y} = \boldsymbol{W} \boldsymbol{\phi}(\boldsymbol{x}) +
    %	\boldsymbol{b},
    % \f]
    % where \f$\boldsymbol{W}\f$ is represented by the variable weights and
    % \f$\boldsymbol{b}\f$ by the variable bias.
    properties (SetAccess = protected)
        featureGenerator = [];
        featureHasHyperParameters = false;
        
        % represents the liniar function as
        % y = weights * x + bias
        weights;
        bias = 0;
        
        
    end
    
    properties (SetObservable, AbortSet)
        initSigmaMu = 0.01;
        doInitWeights = true;
        initMu = [];
    end
    
    methods
        
        
        function obj = FunctionLinearInFeatures(dataManager, outputVariable, inputVariables, functionName, featureGenerator, doInitWeights)
            % @param dataManager Data.Datamanger to operate on
            % @param outputVariable dataset defining the output of the function
            % @param inputVariables dataset defining the input of the function
            % @param functionName name of the function
            % @param featureGenerator feature generator of the dataset
            % @param doInitWeights flag if weights and bias should be initiated in <tt>initObject</tt>
            superargs = {};
            if (nargin >= 1)
                superargs = {dataManager, outputVariable, inputVariables, functionName};
            end
            
            obj = obj@Functions.Mapping(superargs{:});
            obj = obj@Functions.Function();
            
            obj = obj@ParametricModels.ParametricFunction();
            
            if (nargin >= 1)
                if (ischar(outputVariable))
                    obj.linkProperty('initSigmaMu', ['initSigmaMu',  upper(obj.outputVariable(1)), obj.outputVariable(2:end)]);
                    obj.linkProperty('initMu', ['initMu', upper(obj.outputVariable(1)), obj.outputVariable(2:end)]);
                else
                    obj.linkProperty('initSigmaMu');
                    obj.linkProperty('initMu');
                end                
                
                if (exist('featureGenerator', 'var') && ~isempty(featureGenerator) )
                    obj.setFeatureGenerator(featureGenerator);
                else
                    obj.featureGenerator = '';
                end
                
                if (~isempty(obj.inputVariables) && ~isnumeric(obj.inputVariables{1}) && ~iscell(obj.inputVariables{1}))
                    if (isempty(obj.featureGenerator) && dataManager.isFeature(obj.inputVariables{1}))
                        obj.setFeatureGenerator(obj.dataManager.getFeatureGenerator(inputVariables{1}));
                    end
                end
                if(exist('doInitWeights','var'))
                    obj.doInitWeights = doInitWeights;
                end
            end
            obj.registerMappingInterfaceFunction();
            obj.registerGradientFunction();
        end
        
        function [] = initObject(obj)
            % This method will initiate the mapping superclass and the Data.DataManipulator with it.
            %
            % If the doInitWeights flag is set the weight and the bias will also be initialized.
            obj.initObject@Functions.Mapping();
            
            if(obj.doInitWeights)
                obj.bias = zeros(obj.dimOutput,1);
                obj.weights = zeros(obj.dimOutput, obj.dimInput);

                if (isempty(obj.initMu))                                       
                    range = obj.dataManager.getRange(obj.outputVariable);
                    meanRange = (obj.dataManager.getMinRange(obj.outputVariable) + obj.dataManager.getMaxRange(obj.outputVariable)) / 2;
                    
                    obj.bias  = (meanRange' + range' .* obj.initSigmaMu .* (rand(obj.dimOutput, 1) - 0.5));
                    
                else
                    obj.bias  = obj.initMu;
                    if (numel(obj.bias) == 1 && obj.dimOutput > 1)
                        warning('Gaussian Distribution: Initializing mean with scalar, converting in vector\n');
                        obj.bias = repmat(obj.bias, obj.dimOutput, 1);
                    end
                    if (size(obj.bias,2) > 1)
                        obj.bias = obj.bias';
                    end
                end
            end
        end
        
        function [] = setFeatureGenerator(obj, featureGenerator)
            obj.setInputVariables(featureGenerator.outputName);
            obj.featureGenerator = featureGenerator;
            obj.featureHasHyperParameters = isa(featureGenerator, 'Learner.ParameterOptimization.HyperParameterObject');
            obj.addDataManipulationFunction('getExpectationGenerateFeatures', featureGenerator.featureVariables, {obj.outputVariable}, true, true);
        end
        
        function [value] = getExpectationGenerateFeatures(obj, numElements, varargin)
            % Returns the expectation of the Function after generating the Features.
            
            if (~isempty(obj.featureGenerator));
                inputFeatures = obj.featureGenerator.getFeatures(numElements, varargin{:});
            else
                inputFeatures = varargin{1};
            end
            inputFeatures = obj.featureGenerator.getFeatures(numElements, varargin{:});
            value = obj.getExpectation(numElements, inputFeatures);
        end
        
        function [value] = getExpectation(obj, numElements, inputFeatures)
            % Returns the expectation of the Function.
            %
            % If the parameter inputFeatures is not given the function expect
            % it to be zero and only returns the bias. Otherwise this function
            % will return the weighted expectation.
            if (nargin == 2 && obj.dimInput > 0)
                assert('getExpectation function called with only 2 arguments, did you forget the numElements?');
            end
            value = repmat(obj.bias', numElements, 1);
            
            if (nargin == 3 && ~isempty(inputFeatures))
                value = value + inputFeatures * obj.weights';
            end
        end
        
        function [] = setWeightsAndBias(obj, weights, bias)
            obj.bias = bias;
            obj.weights = weights;
            
            if (size(obj.bias,2) > 1)
                obj.bias = obj.bias';
            end
            
            if (size(obj.weights,1) ~= obj.dimOutput)
                obj.weights = obj.weights';
            end
            assert(size(obj.weights,1) == obj.dimOutput && size(obj.weights,2) == obj.dimInput && size(obj.bias,1) == obj.dimOutput);
        end
        
        function [] = setBias(obj, bias)
            obj.bias = bias;
            if (size(obj.bias,2) > 1)
                obj.bias = obj.bias';
            end
        end
        
        %%% Hyper Parameter Functions
        
        function [numParams] = getNumHyperParameters(obj)
            if (obj.featureHasHyperParameters)
                numParams = obj.featureGenerator.getNumHyperParameters();
            else
                numParams = 0;
            end
        end
        
        function [] = setHyperParameters(obj, params)
            if (obj.featureHasHyperParameters)
                obj.featureGenerator.setHyperParameters(params);
            end
        end
        
        function [params] = getHyperParameters(obj)
            if (obj.featureHasHyperParameters)
                params = obj.featureGenerator.getHyperParameters();
            else
                params = [];
            end
        end
        
        function [expParameterTransformMap] = getExpParameterTransformMap(obj)
            if (obj.featureHasHyperParameters)
                expParameterTransformMap = obj.featureGenerator.getExpParameterTransformMap();
            else
                expParameterTransformMap = logical([]);
            end
        end
        
        %%% Parametric Model Function
        function [numParameters] = getNumParameters(obj)
            
            numParameters = dataManager.getNumDimensions(obj.outputVariable) * (1 + dataManager.getNumDimensions(obj.inputVariables));
        end
        
        
        function [gradient] = getGradient(obj, inputMatrix)
            gradient = [ones(size(inputMatrix,1), obj.dimOutput), repmat(inputMatrix, 1, obj.dimOutput)];
        end
        
        
        function [] = setParameterVector(obj, theta)
            obj.bias = theta(1:obj.dimOutput)';
            obj.weights = reshape(theta(obj.dimOutput + (1:(obj.dimInput * obj.dimOutput))), obj.dimOutput, obj.dimInput);
        end
        
        function [theta] = getParameterVector(obj)
            theta = [obj.bias; obj.weights(:)];
        end
        
        function [gradient] = getLikelihoodGradient(obj, varargin)
            assert(false);
        end
        
        
    end
end
