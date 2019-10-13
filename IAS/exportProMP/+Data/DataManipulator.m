classdef DataManipulator < Common.IASObject
    % The DataManipulator class defines the interfaces for manipulating the
    % data structure. Hence this is the base class for almost all classes
    % that interact with the data. It also stores the data manager for the
    % highest hierarchy level such that we can access the properties of the
    % data.
    %
    %@section datamanipulation_function Data manipulation functions
    % Every DataManipulator can publish its data-manipulation functions.
    % For each data manipulation function, we have to specify the input and the output data entries.
    % If we call a function with the data manipulation interface (DataManipulator.callDataFunction),
    % the input arguments get automatically parsed out of the data structure and are put in as matrices
    % for the function call. The output arguments of the function are also automatically
    % stored back in the data structure. Almost every object that is supposed to
    % change a data object is implemented as DataManipulator.
    % This includes sampling episodes from differen environments, policies,
    % learned forward models or reward functions. Data manipulation
    % functions can also obtain the number of elements that need to be processed as the first argument
    % of the call of the function. This is in particularly useful if no
    % other input arguments are specified (for example, for sampling
    % initial states). whether or not a specific data manipulation function
    % get the number of elements as input can be specified by a flag when
    % publishing the function (addDataManipulationFunction).
    %
    %@subsection datamanipulation_call_modes Call modes for DataManipulation functions
    % Data manipulation functions can be called in three different modes.
    % The modes are defined in DataFunctionType and can have the values
    %  - SINGLE_SAMPLE: The data manipulation function is called for each
    %  data point individually (with a for loop).
    %  - ALL_AT_ONCE: The data manipulation function is called for all data
    %  elements at once (in matrix form)
    %  - PER_EPISODE: The data function is called for all data elements
    %  that belong to one episode (i.e. are on the hierarchy level 2).
    %
    %@subsection datamanipulation_calling Calling data manipulation functions
    % The data manipulation functions can be called with
    % callDataFunction or callDataFunctionOutput. The first one also
    % stores the output of the function already in the data structure
    % (note: the output arguments need to be registered in the data manager
    % for that!). With callDataFunctionOutput the output of the data
    % manipulation function is returned as for standard functions. When
    % calling a data manipulation function we can also use the standard
    % hierarchical indicing that we know from the Data class. Hence, the
    % data manipulation functions can be applied to only a subset of the
    % elements of a data object.
    %
    %@section datamanipulation_additionalparameters Datamanipulation function aliases
    % We can also define aliases for data manipulation functions. An alias
    % can point to a single data manipulation function (hence, it serves as
    % a different name for the same data manipulation function), or it can
    % also represent a sequence of data manipulation functions. Please see
    % the function addDataFunctionAlias and the test scripts.
    %

    properties (Access = protected)
        manipulationFunctions;
        samplerFunctions;
        externalSamplers;
        dataManager;
        

    end
    
    methods
        function obj =  DataManipulator(dataManager)
            obj = obj@Common.IASObject();
            if (exist('dataManager', 'var'))
                obj.dataManager = dataManager;
            end
            obj.manipulationFunctions = struct();
            obj.samplerFunctions = containers.Map;
            obj.externalSamplers = containers.Map;
        end
   
        function [] = cloneDataManipulationFunctions(obj, cloneDataManipulator)
            % Clones all data manipulation functions of a given data
            % manipulator.
            obj.manipulationFunctions = cloneDataManipulator.getDataManipulationFunctionsStruct();
            obj.samplerFunctions = cloneDataManipulator.getSamplerFunctionsMap();
            
            % TODO: only works if external sampler is the same object
            functionKeys = obj.samplerFunctions.keys();
            for i = 1:length(functionKeys)
                %if (cloneDataManipulator.externalSamplers(functionKeys{i}) ~= cloneDataManipulator)
                %    error('if you clone, no external sampler is allowed\n');
                %end
                samplingFunctions = obj.samplerFunctions(functionKeys{i});
                if (iscell(samplingFunctions))
                    numSamplingFunctions = length(samplingFunctions);
                else
                    numSamplingFunctions = 1;
                end
                for j = 1:numSamplingFunctions
                    externalSamplersLocal{j} = obj;
                end
                obj.externalSamplers(functionKeys{i}) = externalSamplersLocal;
            end
        end
        
        function [] = addDataFunctionAlias(obj, samplerName, functionName, externalSampler, addInBeginning)
            % Adds a function alias. A function alias is just a different
            % name for the same function. Can be used if certain samplers
            % require a specific data manipulation function that is not
            % directly implemented by the class, but there is another
            % function with the same functionality. For example,
            % used by learned distributions that are used as model or
            % policy. In this case, we can tell the data manipulator that
            % instead of the function sampleAction, we can use
            % sampleFromDistribution. We can also use a data function alias to call
            % several data manipulation function sequentially. If a data
            % function alias has already been registered, the functionName
            % is extended to the cell array of functionNames for that
            % sampler function. If the sampler function is called, all
            % registered data manipulation function for that sampler are
            % called sequentially. We can also specify an external sampler
            % in case we want to call a data manipulation function of
            % another class.
            if(~exist('addInBeginning', 'var') )
                addInBeginning = false;
            end
            if (~exist('externalSampler', 'var') || isempty(externalSampler) )
                externalSampler = obj;
                assert(strcmp(samplerName, functionName) || obj.samplerFunctions.isKey(functionName), sprintf('Error when registering Alias: Function %s is not registered\n', functionName));
            else
                assert(isfield(externalSampler.manipulationFunctions,functionName), sprintf('Error when registering Alias: Function %s is not registered\n', functionName));
            end
            if (obj.samplerFunctions.isKey(samplerName))
                functionList = obj.samplerFunctions(samplerName);
                if (~iscell(functionList))
                    if(addInBeginning)
                        functionList = {functionName, functionList};
                    else
                        functionList = {functionList, functionName};
                    end
                else
                    if(addInBeginning)
                        functionList = {functionName , functionList{:} };
                    else
                        functionList{end + 1} = functionName;
                    end
                end
                obj.samplerFunctions(samplerName) = functionList;
                samplerList = obj.externalSamplers(samplerName);
                if(addInBeginning)
                    samplerList = {externalSampler, samplerList{:}};
                else
                    samplerList{end + 1} = externalSampler;
                end
                
                obj.externalSamplers(samplerName) = samplerList;
            else
                obj.samplerFunctions(samplerName) = functionName;
                obj.externalSamplers(samplerName) = {externalSampler};
            end
        end
        
        function clearDataFunctionAlias(obj, samplerName)
            obj.samplerFunctions.remove(samplerName);
        end
        
        function [dataManipulationFunctions] = getDataManipulationFunctionsStruct(obj)
            dataManipulationFunctions = obj.manipulationFunctions;
        end
        
        function [samplerFunctions] = getSamplerFunctionsMap(obj)
            samplerFunctions = obj.samplerFunctions;
        end
        
        function [dataManager] = getDataManager(obj)
            dataManager = obj.dataManager;
        end
        
        function [dataManipulationFunctions] = getDataManipulationFunctions(obj)
            dataManipulationFunctions = fieldnames(obj.manipulationFunctions);
        end
        
        function [dataManipulationFunctions] = getSamplerFunctions(obj)
            % Returns a list of all data manipulation functions.
            dataManipulationFunctions = obj.samplerFunctions.keys();
        end
        
        function [] = printSamplerFunctions(obj)
            % Prints all data manipulation functions of the current object,
            % including their input and output arguments.
            dataManipulationFunctions = obj.samplerFunctions.keys();
            samplerListValues = obj.samplerFunctions.values();
            
            fprintf('Functions:\n');
            for i = 1:length(dataManipulationFunctions)
                
                
                
                functionName = dataManipulationFunctions{i};
                if (isfield(obj.manipulationFunctions,  functionName))
                    manipulationFunction = obj.manipulationFunctions.(functionName);
                    if (strcmp(dataManipulationFunctions{i}, manipulationFunction.functionName))
                        fprintf('%s: ', dataManipulationFunctions{i});
                        fprintf('{');
                        for j = 1:length(manipulationFunction.inputArguments)
                            if (~iscell(manipulationFunction.inputArguments{j}))
                                fprintf('%s ', manipulationFunction.inputArguments{j});
                            else
                                fprintf('{');
                                for k = 1:length(manipulationFunction.inputArguments{j})
                                    fprintf('%s ', manipulationFunction.inputArguments{j}{k});
                                end
                                fprintf('}');
                            end
                        end
                        fprintf('} -> {');
                        
                        for j = 1:length(manipulationFunction.outputArguments)
                            if (~iscell(manipulationFunction.outputArguments{j}))
                                fprintf('%s ', manipulationFunction.outputArguments{j});
                            else
                                fprintf('{');
                                for k = 1:length(manipulationFunction.outputArguments{j})
                                    fprintf('%s ', manipulationFunction.outputArguments{j}{k});
                                end
                                fprintf('}');
                            end
                        end
                        fprintf('}\n');
                    end
                end
            end
            
            fprintf('Aliases:\n');
            for i = 1:length(dataManipulationFunctions)                                
                
                functionName = dataManipulationFunctions{i};
                if (~isfield(obj.manipulationFunctions,  functionName))
                    samplerFunctionNames = samplerListValues{i};
                    fprintf('%s: -> {', dataManipulationFunctions{i});
                    if ~iscell(samplerFunctionNames)
                        samplerFunctionNames = {samplerFunctionNames};
                    end
                    for j = 1:length(samplerFunctionNames)
                        fprintf('%s ', samplerFunctionNames{j});
                    end
                    fprintf('}\n');
                    
                end
            end
            
        end
        
        function [] = setCallType(obj, name, callType)
            % Sets the call type of the specified data manipulation
            % function. The call type needs to be of the type
            % Data.DataFunctionType. See class description for their
            % meaning.
            dataFunction = obj.manipulationFunctions.(name);
            dataFunction.callType = callType;
            obj.manipulationFunctions.(name)  = dataFunction;
        end
        
        function [] = setTakesData(obj, name, val)
            % Set the call type of the specified data manipulation
            % function. The call type needs to be of the type
            % Data.DataFunctionType. See class description for their
            % meaning.
            if ( ~exist('val','var') || isempty(val) )
                val = false;
            end
            dataFunction = obj.manipulationFunctions.(name);
            dataFunction.takesData = val;
            obj.manipulationFunctions.(name)  = dataFunction;
        end
        
        function [] = setInputIndices(obj, name, numInput, inputIndices)
            % ... hm, better do not use that ...
            dataFunction = obj.manipulationFunctions.(name);
            dataFunction.indices{numInput} = inputIndices;
            obj.manipulationFunctions.(name)  = dataFunction;
        end
        
        function [] = setInputArguments(obj, name, inputArguments)
            % Set the input arguemnts for the specified function. All input
            % arguments MUST be registered at the data manager.
            obj.checkInputArguments(name, inputArguments);
            
            if (~iscell(inputArguments))
                inputArguments = {inputArguments};
            end
            
            dataFunction = obj.manipulationFunctions.(name);
            dataFunction.inputArguments = inputArguments;
            
            obj.manipulationFunctions.(name) = dataFunction;
        end
        
        function [] = setOutputArguments(obj, name, outputArguments)
            % Sets the output arguments of the specified function. The
            % output arguments do not need to be registered at the data
            % manager, however, if we call the data manipulation function with callDataFunction,
            % then all output arguments obviously need to be registered
            % otherwise hell will break out.
            dataFunction = obj.manipulationFunctions.(name);
            if (~iscell(outputArguments))
                outputArguments = {outputArguments};
            end
            dataFunction.outputArguments = outputArguments;
            
            obj.manipulationFunctions.(name) = dataFunction;
        end
        
        function [inputArgs] = getInputArguments(obj, name)
            inputArgs = obj.manipulationFunctions.(name).inputArguments;
        end
        
        function [inputArgs] = getOutputArgs(obj, name)
            inputArgs = obj.manipulationFunctions.(name).outputArguments;
        end
        
        function [isSampler] = isSamplerFunction(obj, samplerName)
            isSampler = obj.samplerFunctions.isKey(samplerName);
        end
        
        function [] = callDataFunction(obj, samplerName, data, varargin)
            % Calls a data manipulation function with the name samplerName.
            % If the samplerName refers to several data manipulation
            % functions, all data manipulation functions are called
            % sequentially.
            % We need to specify the data object. vargargin can be used for
            % the hierarchical indicing of the data structure. The output
            % of the data manipulation function is also stored in the data
            % object under the corresponding data entries.
            if (~obj.samplerFunctions.isKey(samplerName))
                error('callDataFunction: Sampler function %s not found for object\n', samplerName);
            end
            
            functionName = obj.samplerFunctions(samplerName);
            samplerList = obj.externalSamplers(samplerName);
            if (iscell(functionName))
                for i = 1:length(functionName)
                    externalSampler = samplerList{i};
                    
                    if (externalSampler == obj && strcmp(samplerName, functionName{i}))
                        dataManipulationStruct = obj.manipulationFunctions.(functionName{i});
                        externalSampler.callDataFunctionInternal(dataManipulationStruct, data, true, varargin{:});
                    else
                        externalSampler.callDataFunction(functionName{i}, data, varargin{:});
                    end
                end
            else
                externalSampler = samplerList{1};
                if (externalSampler == obj && strcmp(samplerName, functionName))
                    dataManipulationStruct = obj.manipulationFunctions.(functionName);
                    obj.callDataFunctionInternal(dataManipulationStruct, data, true, varargin{:});
                else
                    externalSampler.callDataFunction(functionName, data, varargin{:});
                end
            end
        end
        
        function [varargout] = callDataFunctionOutput(obj, samplerName, data, varargin)
            % Same as callDataFunction just that the output is not stored
            % in the data structure but return as regular return parameters.
            if (~obj.samplerFunctions.isKey(samplerName))
                error('callDataFunction: Sampler function %s not found for object\n', samplerName);
            end
            
            functionName = obj.samplerFunctions(samplerName);
            
            samplerList = obj.externalSamplers(samplerName);
            
            if (iscell(functionName))
                for i = 1:length(functionName)
                    externalSampler = samplerList{i};
                    
                    isLastElement = i == length(functionName);
                    
                    if (externalSampler == obj && strcmp(samplerName, functionName{i}))
                        dataManipulationStruct = obj.manipulationFunctions.(functionName{i});
                        varargout = externalSampler.callDataFunctionInternal(dataManipulationStruct, data, isLastElement, varargin{:});
                    else
                        if (~isLastElement)
                            externalSampler.callDataFunction(functionName{i}, data, varargin{:});
                        else
                            varargout = externalSampler.callDataFunctionOutput(functionName{i}, data, varargin{:});
                        end
                    end
                end
            else
                dataManipulationStruct = obj.manipulationFunctions.(functionName);
                varargout = obj.callDataFunctionInternal(dataManipulationStruct, data, false, varargin{:});
            end
            
            if (~iscell(varargout))
                varargout = {varargout};
            end
        end
        
        function [] = addDataManipulationFunction(obj, functionName, inputArguments, outputArguments, dataFunctionCallType, takesNumElements)
            % Registers a new data manipulation function functionName with
            % inputArguments used for input and outputArguments used for
            % outpt. inputArguments needs to be a cell array of strings. It
            % can also contain other cells, which means that the input
            % argument is a concatenation of several data entries. For
            % example {{'states', 'actions'}, 'rewards'} means that the function gets
            % 2 inputs, the first is a concatenation of states and actions,
            % the 2nd is the rewards. Optional arguments are the
            % dataFunctionCallType (see Data.DataFucntionType) and whether
            % the function obtains the number of elements that needs to be
            % processed as first argument. Note: If the function does not
            % get any inputArguments ({}), then takesNumElements is per
            % default true (as we do not know how many elements we should
            % create). Otherwise, the default is false.
            if (~exist('dataFunctionCallType', 'var'))
                dataFunctionCallType = Data.DataFunctionType.ALL_AT_ONCE;
            end
            if (islogical(dataFunctionCallType))
                if (dataFunctionCallType)
                    dataFunctionCallType = Data.DataFunctionType.ALL_AT_ONCE;
                else
                    dataFunctionCallType = Data.DataFunctionType.SINGLE_SAMPLE;
                end
            end
            
            if ( ~exist('takesNumElements', 'var'))
                takesNumElements = isempty(inputArguments);
            end
            
            %obj.checkInputArguments(functionName, inputArguments);
            
            if (~iscell(inputArguments))
                inputArguments = {inputArguments};
            end
            
            if (~iscell(outputArguments))
                outputArguments = {outputArguments};
            end
            
            dataManipulationFunction = struct();
            dataManipulationFunction.functionName = functionName;
            
            dataManipulationFunction.inputArguments = inputArguments;
            dataManipulationFunction.indices = cell(length(inputArguments), 1);
            dataManipulationFunction.outputArguments = outputArguments;
            
            dataManipulationFunction.callType = dataFunctionCallType;
            
            dataManipulationFunction.takesNumElements = takesNumElements;
            dataManipulationFunction.depthEntry = obj.getDepthEntryForDataManipulationFunction(inputArguments, outputArguments);
            
            dataManipulationFunction.takesData = false;
            
            obj.manipulationFunctions.(functionName) = dataManipulationFunction;
            
            if (obj.samplerFunctions.isKey(functionName))
                obj.samplerFunctions.remove(functionName);
            end
            obj.addDataFunctionAlias(functionName, functionName);
        end
        
        function [depthEntry] = getDepthEntryForDataManipulationFunction(obj, inputArguments, outputArguments)
            depthEntry = '';
            if (~isempty(outputArguments))
                depthEntry = outputArguments{1};
            else
                for i = 1:length(inputArguments)
                    if (~isempty(inputArguments{i}))
                        depthEntry = inputArguments{i};
                        break;
                    end
                end
            end
        end
        
        
    end
    
    methods (Access = protected)
        
        function [outArgs] = callDataFunctionInternal(obj, dataManipulationStruct, data, registerOutput, varargin)
            
            inputArgsIndex = varargin;
            callData = true;
            
            if (dataManipulationStruct.callType == Data.DataFunctionType.PER_EPISODE)
                inputArgsIndex = data.completeLayerIndex(2, inputArgsIndex);
            end
            
            if (dataManipulationStruct.callType == Data.DataFunctionType.SINGLE_SAMPLE)
                %inputArgs = data.completeLayerIndex(inputArgs, );
                % TODO: A proper error handling if we call it with ":" and
                % single samples
            end
            
            if (dataManipulationStruct.callType == Data.DataFunctionType.SINGLE_SAMPLE || dataManipulationStruct.callType == Data.DataFunctionType.PER_EPISODE)
                outArgsTemp = [];
                numLayers = length(inputArgsIndex);
                if (dataManipulationStruct.callType == Data.DataFunctionType.PER_EPISODE)
                    numLayers = 1;
                end
                for i = 1:numLayers
                    if (length(inputArgsIndex{i}) > 1)
                        callData = false;
                        for j = 1:length(inputArgsIndex{i})
                            inputArgsSingle = inputArgsIndex;
                            inputArgsSingle{i} = inputArgsSingle{i}(j);
                            if (registerOutput)
                                obj.callDataFunctionInternal(dataManipulationStruct, data, registerOutput, inputArgsSingle{:});
                            else
                                tempOutArgs = obj.callDataFunctionInternal(dataManipulationStruct, data, registerOutput, inputArgsSingle{:});
                                outArgsTemp = [outArgsTemp; tempOutArgs];
                            end
                        end
                    end
                end
                
                if (~registerOutput)
                    
                    for i = 1:size(outArgsTemp,2)
                        outArgs{i} = cell2mat({outArgsTemp{:,i}}');
                    end
                end
            end
            
            if (callData)
                inputArgs = data.getDataEntryCellArray(dataManipulationStruct.inputArguments, inputArgsIndex{:});
                
                for i = 1:length(inputArgs)
                    if (length(dataManipulationStruct.indices) >= i && ~isempty(dataManipulationStruct.indices{i}))
                        inputArgs{i} = inputArgs{i}(:, dataManipulationStruct.indices{i});
                    end
%                     if (isempty(dataManipulationStruct.inputArguments{i}))
%                         depth = obj.dataManager.getDataEntryDepth(dataManipulationStruct.depthEntry);
%                         inputArgs{i} = zeros(data.getNumElementsForDepth(depth), 0);
%                     end
                end
                
                nOut = length(dataManipulationStruct.outputArguments);
                
                if (dataManipulationStruct.takesNumElements)
                    outputDepth = obj.dataManager.getDataEntryDepth(dataManipulationStruct.depthEntry);
                    numElements = data.getNumElementsForIndex(outputDepth, inputArgsIndex{:});
                    numElements = {numElements};
                else
                    numElements = {};
                end
                [outArgs{1:nOut}] = obj.callDataFunctionInternalMatrices(dataManipulationStruct, numElements, data, inputArgs{:});
                if (registerOutput)
                    data.setDataEntryCellArray(dataManipulationStruct.outputArguments, outArgs, inputArgsIndex{:});
                end
            end
        end
        
        function [varargout] = callDataFunctionInternalMatrices(obj, dataManipulationStruct, numElements, data, varargin)
            nOut = length(dataManipulationStruct.outputArguments);
            if ( dataManipulationStruct.takesData )
                [varargout{1:nOut}] = obj.(dataManipulationStruct.functionName)(numElements{:}, data, varargin{:});
            else
                [varargout{1:nOut}] = obj.(dataManipulationStruct.functionName)(numElements{:}, varargin{:});
            end
        end
        
        function [] = checkInputArguments(obj, functionName, inputArguments)
            for i = 1:length(inputArguments)
                if (~iscell(inputArguments{i}))
                    if (~obj.dataManager.isDataAlias(inputArguments{i}))
                        error('Input argument %s of function %s is not registered yet!', inputArguments{i}, functionName);
                    end
                else
                    for j = 1:length(inputArguments{i})
                        if (~obj.dataManager.isDataAlias(inputArguments{i}{j}))
                            error('Input argument %s of function %s is not registered yet!', inputArguments{i}{j}, functionName);
                        end
                    end
                end
            end
        end
        
        
    end
    
end
