classdef SupervisedLearner < Learner.Learner & Data.DataManipulator & Learner.AbstractInputOutputLearnerInterface
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
        
    end
    
    % Class methods
    methods
        function obj = SupervisedLearner(dataManager, functionApproximator, varargin)
            % @param dataManager Data.DataManger to operate on
            % @param functionApproximator function object that will be learned
            % @param weightName name of the weight
            % @param inputVariables set of input Variables of the functionApproximator
            % @param outputVariable set of outputVariables of the functionApproximator this set should only contain one variable
            obj = obj@Learner.Learner();
            obj = obj@Data.DataManipulator(dataManager);
            obj = obj@Learner.AbstractInputOutputLearnerInterface(dataManager, functionApproximator, varargin{:});
                       
        end
        
        function [] = updateModel(obj, data)
            % alternate function call for learnFunction()
            obj.callDataFunction('learnFunction', data);
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
        
        function [outputVariable] = getOutputVariable(obj)
            outputVariable = obj.outputVariable;
        end
        
    end
    
    methods(Abstract)
        [] = learnFunction(obj, inputData, outputData, varargin);
    end
    
    methods (Access = protected)
        function [] = registerLearnFunction(obj)                            
            if (isempty(obj.inputVariables))
                inputVariablesTemp = {''};
            else
                inputVariablesTemp = obj.inputVariables;
            end
            obj.addDataManipulationFunction('learnFunction', {inputVariablesTemp{:}, obj.outputVariable, obj.additionalInputArguments{:}, obj.weightName{:}}, {});            
        end        
   end
end
