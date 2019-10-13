classdef AbstractInputOutputLearnerInterface < Common.IASObject
    % The SupervisedLearner serves as a base class for all supervised learners. 
    %
    % This class provides the basic properties of each supervised learner
    % like the function approximator that handles the function class that 
    % models the to-learn function and the weights. 
    % Be sure that the function approximator only has one outputVariable if you are using the functions provided by the policytoolbox:
    % 
    %
    % Optionally you are able to set new input and output Variables for your function approximator in the constructor.
    %
    % The abstract function <tt>learnFunction</tt> will determine the way 
    % the supervised learner calculates the function given by the functionApproximator.
    
    properties
        functionApproximator;
        additionalInputArguments = {};
        weightName
        
        inputVariables;
        outputVariable;
        
    end
    
    % Class methods
    methods
        function obj = AbstractInputOutputLearnerInterface(dataManager, functionApproximator, weightName, inputVariables, outputVariable)
            % @param dataManager Data.DataManger to operate on
            % @param functionApproximator function object that will be learned
            % @param weightName name of the weight
            % @param inputVariables set of input Variables of the functionApproximator
            % @param outputVariable set of outputVariables of the functionApproximator this set should only contain one variable
                       
            obj.functionApproximator = functionApproximator;
                                    
            if (exist('weightName', 'var') && ~isempty(weightName))
                obj.weightName = {weightName};
            else
                obj.weightName = {};
            end
            if (isempty(obj.functionApproximator))
                assert(exist('inputVariables', 'var') && exist('outputVariable', 'var'), 'pst:Supervised Learner: If no function approximator is provided you need to pass input and output Variables!');
            end
            
            if (exist('inputVariables', 'var'))
                obj.inputVariables = inputVariables;
                if (~iscell(obj.inputVariables))
                    obj.inputVariables = {obj.inputVariables};
                end
            else
                obj.inputVariables = obj.functionApproximator.inputVariables;
            end
            
            if (exist('outputVariable', 'var'))
                obj.outputVariable = outputVariable;
            else
                obj.outputVariable = obj.functionApproximator.outputVariable;
            end
            
            obj.setAdditionalInputArguments();
                                                
            obj.registerLearnFunction();
        end
        
        function [] = setInputVariablesFromMapping(obj)
            if (~isempty(obj.functionApproximator))
                obj.inputVariables = obj.functionApproximator.inputVariables;
                obj.outputVariable = obj.functionApproximator.outputVariable;
                obj.registerLearnFunction();
            end
                
        end
              
        function [] = setWeightName(obj, weightName)
            obj.weightName = {weightName};
            obj.registerLearnFunction();
        end    
        
        function [isWeight] = isWeightedLearner(obj)
            isWeight = ~isempty(obj.weightName);
        end
        
        function [weightName] = getWeightName(obj)
            weightName = obj.weightName{1};
        end
                
        function [] = setInputVariablesForLearner(obj, varargin)
            obj.inputVariables = varargin;
            obj.registerLearnFunction();
        end
        
        function [] = setOutputVariableForLearner(obj, outputVariable)
            obj.outputVariable = outputVariable;
            obj.registerLearnFunction();
        end
        
        function [] = setFunctionApproximator(obj, funcApprox)
            obj.functionApproximator = funcApprox;
        end

        
    end
    
    methods (Access = protected)
        function [] = registerLearnFunction(obj)                            
        end
        
        function [] = setAdditionalInputArguments(obj, varargin)
            if (~isempty(obj.functionApproximator))
                obj.additionalInputArguments  = obj.functionApproximator.additionalInputVariables;
            else
                obj.additionalInputArguments = {};
            end
            obj.additionalInputArguments = {obj.additionalInputArguments{:}, varargin{:}};
            obj.registerLearnFunction();
        end
    end
end
