classdef Mapping < Data.DataManipulator & dynamicprops
    % The Mapping class is a Data.DataManipulator that is able to combine a
    % number of data manipulation functions.
    %
    % Every Mapping contains a set of data manipulation function as well as
    % sets for the input and output variables. The input and output can be 
    % defined in the constructor or at a later point via <tt>setInputVariables()</tt>
    % and <tt>setOutputVariables()</tt>. New mapping functinos have to be 
    % added with the <tt>addMappingFunction()</tt>.
    properties (SetAccess = protected)
        mappingName % Name of this mapping
        
        inputVariables = {}; % collection of input Variables
        additionalInputVariables = {};
        outputVariable = {}; % collection of output Variables
        
        dimInput
        dimOutput
        
        mappingFunctions = {}; % collection of mapping functions
        mappingFunctionsOutputVariables = {}; % collection of the output variables of the functions, not necessarily equal to outputVariable
        
        registeredMappingFunctions = false; % flag indicating if any mapping functions have been included in this Functions.Mapping
        registerDataFunctions = true;
    end
    
    methods
        function obj = Mapping(dataManager, outputVariable, inputVariables, mappingName)
            % @param dataManager Data.DataManager this mapping operates
            % @param outputVariable name of the data entry to which this mapping will output 
            % @param inputVariables name of the data entry from which this mapping will get input 
            % @param mappingName name of this mapping (Default: 'function')
            superargs = {};
            if (nargin >= 1)
                superargs = {dataManager};
            end
            
            obj = obj@Data.DataManipulator(superargs{:});
            obj.mappingName = 'function';
            
            if (nargin >= 1)
                if (nargin >= 2)
                    obj.setOutputVariables(outputVariable);
                    if (nargin >= 3)
                        if (~iscell(inputVariables))
                            obj.setInputVariables(inputVariables);
                        else
                            obj.setInputVariables(inputVariables{:});
                        end
                    end
                    if (nargin >= 4)
                        obj.mappingName = mappingName;
                    else
                        if (ischar(outputVariable(1)))
                            obj.mappingName = [upper(outputVariable(1)), outputVariable(2:end)];
                        else
                            obj.mappingName = 'UnknownMapping';
                        end
                    end
                end
            end
        end
        
        function [] = setAdditionalInputVariables(obj, varargin)
            obj.additionalInputVariables = varargin;
        end
        
        function [] = addMappingFunction(obj, mappingFunctionName, outputVariables)
            % @param mappingFunctionName Name of the new mapping function
            % @param outputVariables optional new output variables
            %
            % by adding a new mapping function the Mapping will register 
            % a new DataManipulationFunction in the Data.DataManager, with 
            % the currently defined inputVariables and the current set of
            % outputVariables also including the new outputVariables added
            % in this function call. (see also Data.DataManipulator)
            if (nargin < 3)
                outputVariables = {''};
            end
            if (~iscell(outputVariables))
                outputVariables = {outputVariables};
            end
            
            obj.mappingFunctions{end + 1} = mappingFunctionName;
            obj.mappingFunctionsOutputVariables{end + 1} = {};
            
            for i = 1:length(outputVariables)
                obj.mappingFunctionsOutputVariables{end}{i} = [obj.outputVariable, outputVariables{i}];
            end            
            

            if (~isempty(obj.inputVariables))
                if (~isnumeric(obj.inputVariables{1}))
                    
                    obj.addDataManipulationFunction(obj.mappingFunctions{end}, [obj.inputVariables, obj.additionalInputVariables], obj.mappingFunctionsOutputVariables{end}, true, true);
                end
            else
                obj.addDataManipulationFunction(obj.mappingFunctions{end}, [obj.additionalInputVariables], obj.mappingFunctionsOutputVariables{end}, true, true);
            end
            
        end
        
        function [depthEntry] = getDepthEntryForDataManipulationFunction(obj, ~, ~)
            depthEntry = obj.outputVariable;
        end
        
        function [] = registerMappingFunction(obj)
            obj.registeredMappingFunctions=true;
            
            if (~isnumeric(obj.inputVariables{1}))
                for i = 1:length(obj.mappingFunctions)
                    obj.addDataManipulationFunction(obj.mappingFunctions{i}, [obj.inputVariables, obj.additionalInputVariables{:}], [obj.outputVariables{1}, obj.mappingFunctionsOutputVariables{i}], true, true);
                end
            end
        end
        
        function [] = setMappingName(obj, name)
            obj.mappingName = name;
        end
        
        function [] = setInputVariables(obj, varargin)
            obj.inputVariables = varargin;
            
            if (~isempty(obj.inputVariables))
                if (~isnumeric(obj.inputVariables{1}))
                    obj.dimInput = obj.dataManager.getNumDimensions(obj.inputVariables);

                    if (isempty(obj.inputVariables) || isempty(obj.inputVariables{1}))
                        obj.inputVariables = {};
                    end

                    for i = 1:length(obj.mappingFunctions)
                        obj.setInputArguments(obj.mappingFunctions{i}, obj.inputVariables, obj.additionalInputVariables{:});
                    end
                else
                    obj.dimInput = obj.inputVariables{1};
                    obj.registerDataFunctions = false;
                end
            else
                obj.registerDataFunctions = false;
%                obj.dimInput = obj.inputVariables{1};
                obj.dimInput = 0;
            end
        end
        
        function [inputVariable] = getInputVariable(obj, index)
            inputVariable = obj.inputVariables{index};
        end
        
        function [] = setOutputVariables(obj, outputArgument)
            
            if(isnumeric(outputArgument))
                obj.dimOutput = outputArgument;
                obj.outputVariable = {};

                if (obj.registerDataFunctions)
                    for i = 1:length(obj.mappingFunctions)
                        obj.setOutputArguments(obj.mappingFunctions{i}, [obj.mappingFunctionsOutputVariables{i}]);
                    end
                end
            else
                obj.outputVariable = outputArgument;
                
                obj.dimOutput = obj.dataManager.getNumDimensions(obj.outputVariable);
                
                for i = 1:length(obj.mappingFunctions)
                    obj.setOutputArguments(obj.mappingFunctions{i}, [outputArgument, obj.mappingFunctionsOutputVariables{i}]);
                end
            end            
        end
        
        function [outputVariable] = getOutputVariable(obj)
            outputVariable = obj.outputVariable;
        end
        
        
        function [] = cloneDataManipulationFunctions(obj, cloneDataManipulator)
            obj.cloneDataManipulationFunctions@Data.DataManipulator(cloneDataManipulator);
            obj.inputVariables = cloneDataManipulator.inputVariables;
            obj.outputVariable = cloneDataManipulator.outputVariable;
            obj.dimInput = obj.dataManager.getNumDimensions(obj.inputVariables);
            obj.dimOutput = obj.dataManager.getNumDimensions(obj.outputVariable);
        end
        
        
    end
    
    methods (Abstract)
        
    end
    
end
