classdef Distribution < Functions.MappingInterface
    % The Distribution class is the base class for distributions.
    %
    % The functions <tt>setDataProbabilityEntries()</tt> and 
    % <tt>registerProbabilityNames()</tt> can be used to create and register
    % Dataentries that contain correlation of two sets of data.
    %
    % This class registers the abstract functions <tt>'sampleFromDistribution'</tt> 
    % and <tt>'getDataProbabilities'</tt>. 
    %
    % The to create a subclass of distribution you need to define the 
    % following abstract classes: 
    % 
    % - <tt>sampleFromDistribution(obj, numElements, varargin)</tt> : should
    % return a matrix of numElements many random samples from this distribution.
    % - <tt>getDataProbabilities(obj, varargin)</tt>: should return the 
    % log likelihood for a given set of input data and output data to be related 
    % in this distribution.
    properties (SetAccess = protected)
        dataProbabilityEntries;
    end
    
    methods
        function obj = Distribution()
            obj = obj@Functions.MappingInterface();

        end
        
        
        
        function [] = setDataProbabilityEntries(obj)
            % This function will create a new ProbabilityEntries to the 
            % dataProbabilityEntries list. The Dataentry will be a combined
            % string <tt>’logQ’ + <uppercase of the first letter of the output 
            % variable> +<lowercase of the first letter of the input variable></tt>.
            % The list of data probability entries can be registered via 
            % <tt>registerProbabilityNames()</tt>.
            inputVariablesShort = '';
            outputVariablesShort = '';
            
            for i = 1:length(obj.inputVariables)
                if (iscell(obj.inputVariables{i}))
                    for j = 1:length(obj.inputVariables{i})
                        inputVariablesShort = [inputVariablesShort, lower(obj.inputVariables{i}{j}(1))];
                    end
                else
                   inputVariablesShort = [inputVariablesShort, lower(obj.inputVariables{i}(1))];
                end
            end
                        
            outputVariablesShort = [outputVariablesShort, upper(obj.outputVariable(1))];
                        
            obj.dataProbabilityEntries{1} = ['logQ', outputVariablesShort, inputVariablesShort];
        end
        
        function [] = registerProbabilityNames(obj, dataManager, layerName)
            % registers all data probability entrys on the dataProbabilityEntries
            % list, created via <tt>setDataProbabilityEntries()</tt>
            for i = 1:length(obj.dataProbabilityEntries)
                dataManager.addDataEntry([layerName, '.', obj.dataProbabilityEntries{i}], 1);
            end
        end
        
        function [dataProbabilityNames] = getDataProbabilityNames(obj)
             dataProbabilityNames = obj.dataProbabilityEntries{1};
        end
    end
    
    methods (Access=protected)
        function [] = registerMappingInterfaceDistribution(obj)
            if (obj.registerDataFunctions)
                obj.addMappingFunction('sampleFromDistribution');
                if (~isempty(obj.outputVariable))
                    obj.setDataProbabilityEntries();
                    if (obj.registerDataFunctions)
                        obj.addDataManipulationFunction('getDataProbabilities', [obj.inputVariables, obj.outputVariable, obj.additionalInputVariables{:}], obj.dataProbabilityEntries);        
                    end
                end
            end
        end
    end
                
    methods (Abstract)
        % - <tt>sampleFromDistribution(obj, numElements, varargin)</tt> : should
        % return a matrix of numElements many random samples from this distribution.
        % - <tt>getDataProbabilities(obj, varargin)</tt>: should return the 
        % log likelihood for a given set of input data and output data to be related 
        % in this distribution. 
        [value] = sampleFromDistribution(obj, numElements, varargin)
        [varargout] = getDataProbabilities(obj, varargin)
    end    
end
