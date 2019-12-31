classdef Data < Data.AbstractData
    % The Data class is essential for the data management and represents a
    % single data set. The data is organized hierarchically, where the
    % strucuture of the data is given by the hierarchical data manager
    % (see Data.DataManager).
    % The data class provides functions to access and set the hierarchical
    % data in an easy way by using standard matlab indicing notation. So
    % far, a maximum of three levels of hierarchy have been implemented.
    % The data is stored in the property dataStructure, which represents
    % our hierarchical structure. At each level of the hierarchy, this data
    % structure contains a matrix for each data entry. Each matrix has to
    % have the same number of rows (stands for the number of samples). For
    % each sample in the current hierarchy, there is also a data entry that
    % represents the data of the next level of hierarchy. These data entries
    % are stored as structure arrays. The name of the data entries are
    % given in the data manager. The name of the next layer of hierarchy is
    % given by the name of the data manager for the next hierarchy.
    %
    % For example, if we have 2 data managers, one for the 'episodes' and
    % one for the 'steps' within the episodes, the dataStructure property
    % will contain elements such as .parameters which are valid for one
    % episode.
    % With dataStructure.steps(1).actions we can access all actions of the
    % first episode. However, this is only the internal representation. It
    % is recommendated to use the data access functions such as
    % getDataEntry for getting and setting the data fields. For an example
    % how to use the data structure and the data managers please consult
    % the test scripts "+tests/testDataManager.m" and
    % "+tests/testDataManagerAlias.m".
    %
    %@section data_indicing Hierarchical indicing
    % The data can be set and retrieved with the functions setDataEntry and
    % getDataEntry. For both functions, we can use hiearchical indicing.
    % The indicing follows the basic matlab rules. After specifying the
    % name of the data entry and additional arguments (for example the
    % values for setting a data entry), we can specify the index list. The
    % indices are optional parameters that specify the index for each layer
    % of the hierarchy. They can be single numbers, vectors or a ":"
    % symbol, standing for all elements. If an index for a certain layer is
    % not specified, the symbol ":" is used as standard. Indicing works for
    % data entries as well as data aliases. Examples for indicing are
    %
    % -get the first action of all episodes
    %
    %       actions = data.getDataEntry('actions', :, 1)
    %
    % -get the first subAction of the first 5 steps of the first 5 episodes
    %
    %       subActions = data.getDataEntry('actions', 1:5, 1:5, 1)
    %
    %@section data_functionality Other functionality
    % The data object also provides functionality for obtaining the number
    % of samples for a certain hierarchy (also using indicing) or getting
    % several data entries at once (getDataEntryCellArray)
    
    properties (SetAccess = protected)
        dataStructure;
        dataManagerNames
        dataEntries
        featureNames = {};
    end
    
    properties (Access = private)
        
    end
    
    methods
        
        function [obj] = Data(dataManager, dataStructure)
            % Gets the data manager and the data structure that it
            % encapsulated. Is typically called by the data manager. Use
            % DataManager.getDataObject to create a new data object. Stores
            % the data structure and also constructs several maps for
            % faster data access.
            obj = obj@Data.AbstractData(dataManager);
            obj.dataManager = dataManager;
            obj.dataStructure = dataStructure;
            
            obj.initDataStructureEntries();
        end
        
        function [dataManager] = getDataManager(obj)
            dataManager = obj.dataManager;
        end
        
        function [layerIndex] = completeLayerIndex(obj, targetLayer, layerIndex)
            % This function completes the hierarchical indicing for the
            % internal functions for accessing the data. E.g., if not
            % enough indices are specified, it appends the ":" symbols
            % (index for every data point).
            if (length(layerIndex) > targetLayer)
                layerIndex = layerIndex(1:targetLayer);
                warning('policysearchtoolbox:Too Many Layers Provided in Data Layer Index, Ignoring...');
            else
                [layerIndex{end + 1:targetLayer}] = deal(':');
            end
            
            for i = 1:(length(layerIndex) - 1)
                if (ischar(layerIndex{i}) && layerIndex{i} == ':')
                    layerIndexLocal = layerIndex(1:i);
                    numElements = obj.getNumElementsForIndex(length(layerIndexLocal), layerIndexLocal{:});
                    layerIndex{i} = 1:numElements;
                end
            end
        end
        
        function [dataStructure] = getDataStructure(obj)
            dataStructure = obj.dataStructure;
        end
        
        function [isDataEntry] = isDataEntry(obj, elementName)
            isDataEntry = obj.dataManager.isDataAlias(elementName);
        end
        
        function [dataStructure] = getDataStructureForLayer(obj, depth)
            [layerIndex{1:depth - 1}] = deal(':');
            dataStructure = obj.getDataStructureForLayerIndex(layerIndex{:});
        end
        
        function [dataStructure] = getDataStructureForLayerIndex(obj, varargin)
            % Returns the data structure for the given layer index. Needed
            % by most functions for fast access of the data.
            
            layerIndex = varargin;
            
            
            if (isempty(layerIndex))
                dataStructure = obj.dataStructure;
            else
                dataManager = obj.dataManager.getSubDataManager();
                dataManagerName = dataManager.getManagerName();
                
                if (ischar(layerIndex{1}) && layerIndex{1}(1) == ':')
                    dataStructure = obj.dataStructure.(dataManagerName)(:);
                    if (~isempty(dataStructure))
                        numElements = sum([dataStructure(:).numElements]);
                    else
                        numElements = 0;
                    end
                else
                    dataStructure = obj.dataStructure.(dataManagerName)(layerIndex{1});
                end
                
                if (numel(dataStructure) > 0)
                    for i = 2:length(layerIndex)
                        dataManager = dataManager.getSubDataManager();
                        dataManagerName = dataManager.getManagerName();
                        if (~(ischar(layerIndex{i}) && layerIndex{i}(1) ~= ':'))
                            newDataStructure = dataStructure(1).(dataManagerName)(layerIndex{i});
                            counter = 1;
                            for j = 2:length(dataStructure)
                                counter = counter + 1;
                                newDataStructure = [newDataStructure; dataStructure(j).(dataManagerName)(layerIndex{i})];
                            end
                        else
                            %                             numElements = sum([dataStructure.numElements]);
                            %                             newDataStructure = reshape([dataStructure(:).(dataManagerName)], numElements, 1);
                            newDataStructure = cell2mat({dataStructure(:).(dataManagerName)}');
                        end
                        dataStructure = newDataStructure;
                    end
                end
            end
            if (size(dataStructure,2) > 1)
                dataStructure = dataStructure';
            end
        end
        
        function [] = setDataStructureForLayer(obj, dataStructure, depth)
            [layerIndex{1:depth - 1}] = deal(':');
            obj.setDataStructureForLayerIndex(dataStructure, layerIndex{:});
        end
        
        function [] = setDataStructureForLayerIndex(obj, dataStructure, varargin)
            layerIndex = varargin;
            switch size(layerIndex,2)
                case 0
                    obj.dataStructure = dataStructure;
                    
                case 1
                    dataManager = obj.dataManager.getSubDataManager();
                    dataManagerName = dataManager.getManagerName();
                    if (ischar(layerIndex{1}) && layerIndex{1}(1) == ':' )
                        obj.dataStructure.(dataManagerName) = dataStructure;
                    else
                        obj.dataStructure.(dataManagerName)(layerIndex{1}) = dataStructure;
                    end
                    
                case 2
                    dataManager = obj.dataManager.getSubDataManager();
                    dataManagerName = dataManager.getManagerName();
                    dataManager2 = dataManager.getSubDataManager();
                    dataManagerName2 = dataManager2.getManagerName();
                    
                    if (ischar(layerIndex{1}) && layerIndex{1}(1) == ':')
                        layerIndexLocal1 = 1:obj.dataStructure.numElements;
                    else
                        layerIndexLocal1 = layerIndex{1};
                    end
                    
                    if (ischar(layerIndex{2}) && layerIndex{2}(1) == ':')
                        [~, numElementsList] = obj.getNumElementsForDepth(2);
                    else
                        numElementsList = ones(obj.getNumElementsForDepth(1),1) * length(layerIndex{2});
                        
                    end
                    
                    
                    index = 1;
                    numElementsList = numElementsList(layerIndexLocal1);
                    for i = 1:length(layerIndexLocal1)
                        
                        obj.dataStructure.(dataManagerName)(layerIndexLocal1(i)).(dataManagerName2)(layerIndex{2}) = dataStructure(index:(index + numElementsList(i) - 1));
                        index = index + numElementsList(i);
                    end
                    assert(index - 1 == length(dataStructure)); % this should always be true
                    
                otherwise
                    assert(false, 'Level 4 is not implemented in the data hierarchy!');
            end
        end
        
        function [] = reserveStorage(obj, numElements, varargin)
            obj.reserveStorageInternal(numElements, true, varargin{:});
        end
        
        function [] = reserveStorageNoReserveOld(obj, numElements, varargin)
            obj.reserveStorageInternal(numElements, false, varargin{:});
        end
        
        function [] = reserveStorageInternal(obj, numElements, reserveOld, varargin)
            % Reserves storage for the data structure. All data entries get
            % initialized at the specified size. The function works
            % hierarchically, where numElements can be a vector containing
            % the numElements for each layer of the hiearchy. varargin can
            % be the index for the hierarchy in case only a part of the
            % data structure should be expanded.
            if (length(numElements)>1)
                layerIndex = varargin;
                for i = 1:length(numElements)
                    [~, numElementsList] = obj.getNumElementsForIndex(i + length(varargin), layerIndex{1:min(i, length(layerIndex))});
                    obj.reserveStorageInternal(numElements(i), reserveOld, layerIndex{:});
                    if (reserveOld)
                        layerIndex{end + 1} = ':';
                    else
                        layerIndex{end + 1} = (numElementsList(1) + 1):numElements(i);
                    end
                end
            else
                dataStructure = obj.getDataStructureForLayerIndex(varargin{:});
                depth  = length(varargin) + 1;
                dataManager = obj.dataManager.getDataManagerForDepth(depth);
                
                for i = 1:length(dataStructure)
                    if (dataStructure(i).numElements > numElements)
                        dataStructure(i) = dataManager.removeLastNElements(dataStructure(i), dataStructure(i).numElements - numElements);
                    elseif (dataStructure(i).numElements < numElements)
                        dataStructure(i) = dataManager.addDataPoints(dataStructure(i), numElements - dataStructure(i).numElements);
                    end
                end
                obj.setDataStructureForLayerIndex(dataStructure, varargin{:});
            end
        end
        
        function [] = copyValuesFromDataStructure(obj, copyDataStructure)
            % This function copies all values from the copyDataStructure
            % that have the same data entries as the given once. This
            % function can be used to create a data object from the
            % dataStructures that are stored in the trial and iter files
            % from the Experiment management
            obj.dataStructure = obj.dataManager.copyValuesFromDataStructure(obj.dataStructure, copyDataStructure);
        end
        
        function [] = mergeData(obj, data, inBack)
            % Merges two data objects. The data from the second data object
            % is either added in the back (inBack = true) or in the front
            % of the data points of the current data object. inBack is
            % optional (true standard)
            if (~exist('inBack', 'var'))
                inBack = true;
            end
            
            dataStructureOther = data.getDataStructure();
            if (inBack)
                obj.dataStructure = obj.dataManager.mergeDataStructures(obj.dataStructure, dataStructureOther);
            else
                obj.dataStructure = obj.dataManager.mergeDataStructures(dataStructureOther, obj.dataStructure);
            end
        end
        
        function [] = mergeDataStructure(obj, dataStructureOther, varargin)
            dataStructure = obj.getDataStructureForLayerIndex(varargin{:});
            depth  = length(varargin) + 1;
            dataManager = obj.dataManager.getDataManagerForDepth(depth);
            
            dataStructure = dataManager.mergeDataStructure(dataStructure, dataStructureOther);
            obj.setDataStructureForLayerIndex(dataStructure, varargin{:});
        end
        
        function [] = removeLastNElements(obj, numElements, varargin)
            % Removes the last N elements. Again works hierarchically
            % including the hierarchical indicing.
            dataStructure = obj.getDataStructureForLayerIndex(varargin{:});
            depth  = length(varargin) + 1;
            dataManager = obj.dataManager.getDataManagerForDepth(depth);
            
            dataStructure = dataManager.removeLastNElements(numElements, dataStructure);
            obj.setDataStructureForLayerIndex(dataStructure, varargin{:});
        end
        
        function [] = deleteData(obj, keepIndices, varargin)
            % Deletes the elements from the data object that are not
            % contained in keepIndices. varargin can again be a
            % hierarchical index that selects only a sub-part of the data
            % structure (or a lower level).
            dataStructure = obj.getDataStructureForLayerIndex(varargin{:});
            depth  = length(varargin) + 1;
            dataManager = obj.dataManager.getDataManagerForDepth(depth);
            
            dataStructure = dataManager.deleteDataIndex(dataStructure, keepIndices);
            obj.setDataStructureForLayerIndex(dataStructure, varargin{:});
        end
        
        function [] = addDataPoints(obj, numElements, varargin)
            % Reserves more storage for the data structure.
            dataStructure = obj.getDataStructureForLayerIndex(varargin{:});
            depth  = length(varargin) + 1;
            dataManager = obj.dataManager.getDataManagerForDepth(depth);
            
            dataStructure = dataManager.addDataPoints(numElements, dataStructure);
            obj.setDataStructureForLayerIndex(dataStructure, varargin{:});
        end
        
        function [numElements] = getNumElements(obj, elementName)
            % Returns the number of elements for the given data entry
            % (depends on the hierarchy level of the data entry).
            if (~exist('elementName', 'var'))
                depth = 1;
            else
                depth = obj.dataEntries.(elementName).depth;
            end
            
            numElements = obj.getNumElementsForDepth(depth);
        end
        
        function [maxDepth] = getMaxDepth(obj)
            % Gets the maximum level of the hierarchy.
            maxDepth = length(obj.dataManagerNames);
        end
        
        function [numElements, numElementsList] = getNumElementsForDepth(obj, depth)
            % Returns the number of elements for the given depth of the
            % hierarchy. Also returns a list with the number of elements
            % for each individual element on this hierarchy. For examples,
            % if depth = 2, it returns a list with the number of steps for
            % each episode.
            [layerIndex{1:depth - 1}] = deal(':');
            
            dataStructure = obj.getDataStructureForLayerIndex(layerIndex{:});
            
            numElementsList = [dataStructure(:).numElements];
            numElements = sum(numElementsList);
        end
        
        function [numElements, numElementsList] = getNumElementsForIndex(obj, depth, varargin)
            % Returns the number of elements for the specified hierarchical
            % index in varargin.
            layerIndex = varargin;
            
            setLastIndex = false;
            
            if (length(layerIndex) < depth)
                [layerIndex{(1 + length(layerIndex)):depth}] = deal(':');
            end
            
            if (length(layerIndex) >= depth)
                lastIndex = layerIndex{depth};
                layerIndex = {layerIndex{1:depth - 1}};
                setLastIndex = true;
            end
            
            dataStructureTemp = obj.getDataStructureForLayerIndex(layerIndex{:});
            
            if (setLastIndex)
                if (ischar(lastIndex) && lastIndex == ':')
                    numElementsList = dataStructureTemp.numElements;
                    numElements = sum([dataStructureTemp.numElements]);
                else
                    if (islogical(lastIndex))
                        numElements = size(dataStructureTemp,1) * sum(lastIndex);
                    else
                        numElements = size(dataStructureTemp,1) * length(lastIndex);
                    end
                    numElementsList = numElements;
                end
            else
                numElements = size(dataStructureTemp,1);
                numElementsList = numElements;
            end
        end
        
        function [dataMatrix] = getDataEntry(obj, aliasName, varargin)
            % Returns the data points from the required data entry (or
            % alias). varargin is a hierarchical index (depending on the
            % hierarchy, it can have different number of elements). For a
            % better understanding of hierarchical indicing pls read the
            % docu of the base class and consult the test files.
            if (isempty(aliasName))
                numElements = obj.getNumElementsForIndex(obj.getMaxDepth(), varargin{:});
                dataMatrix = zeros(numElements, 0);
                return;
            end
            
            if (iscell(aliasName))
                dataMatrix = [];
                for i = 1:length(aliasName)
                    dataMatrix = [dataMatrix, obj.getDataEntry(aliasName{i}, varargin{:})];
                end
                return;
            end
            
            [elementNames, indexList, queryStructArray] = obj.checkDataAliasFeatures(aliasName, varargin{:});
            %depth = obj.dataManager.getDataEntryDepth(elementNames{1});
            
            if (isempty(elementNames))
                numElements = obj.getNumElementsForIndex(obj.dataManager.getDataEntryDepth(aliasName), varargin{:});
                dataMatrix = zeros(numElements, 0);
                return;
            end
            
            indexOffset = 0;
            dataMatrix = obj.getDataEntryInternal(queryStructArray(1));
            
            for i = 2:length(elementNames)
                dataTemp = obj.getDataEntryInternal(queryStructArray(i));
                
                if (ischar(indexList{i}))
                    indexData = 1:obj.dataEntries.(elementNames{i}).numDimensions;
                else
                    indexData = indexList{i};
                end
                
                dataMatrix = [dataMatrix, dataTemp];
                indexOffset = indexOffset + length(indexData);
            end
            
            %             if (depth + 1 == length(varargin))
            %                 dataMatrix = dataMatrix(:, varargin{end});
            %             end
            
            %dataMatrix = zeros(obj.getNumElementsForIndex(obj.dataEntries.(aliasName).depth, varargin{:}), obj.dataEntries.(aliasName).numDimensions);
        end
        
        function [dataMatrix] = getDataEntryFlatIndex(obj, aliasName, flatIndex)
            % Returns the data points for a flat index vector. All elements
            % from the hierarchy are concatenated and then the flat index
            % is used to select the data points. For example, if we have
            % 100 episodes with 50 steps, the flatIndex can select out of
            % 5000 data points. The flat index can either be a logical (of
            % 5000 elements) or a vector of indices.
            assert(~ischar(flatIndex), 'Using : is not supported for the flat index!');
            
            if (islogical(flatIndex))
                numData = sum(flatIndex);
            else
                numData = length(flatIndex);
            end
            dataMatrix = zeros(numData, obj.dataEntries.(aliasName).numDimensions);
            [elementNames, indexList] = obj.checkDataAliasFeatures(aliasName);
            
            indexOffset = 0;
            for i = 1:length(elementNames)
                dataTemp = obj.getDataEntryFlatIndexInternal(elementNames{i}, indexList{i}, flatIndex);
                
                if (ischar(indexList{i}))
                    indexData = 1:obj.dataEntries.(elementNames{i}).numDimensions;
                else
                    indexData = indexList{i};
                end
                
                dataMatrix(:, indexData + indexOffset) = dataTemp;
                indexOffset = indexOffset + length(indexData);
            end
        end
        
        function [] = setDataEntry(obj, aliasName, dataVector, varargin)
            % Sets the data points for a certain data entry (or alias). The
            % data points are selected by the hierarchical indicing
            % (varargin). For a better understanding of hierarchical
            % indicing, please read the documentation of the base class and
            % consult the test files.
            if (~isfield(obj.dataEntries, aliasName))
                error('Fieldname %s is not an entry or alias\n', aliasName);
            end
            if (size(dataVector, 2) ~= obj.dataEntries.(aliasName).numDimensions)
                error('Set data has wrong dimensionality for data entry %s (%d instead of %d)\n', aliasName, size(dataVector, 2), obj.dataEntries.(aliasName).numDimensions);
            end
            
            elementNames = obj.dataEntries.(aliasName).entryNames;
            indexList = obj.dataEntries.(aliasName).indexList;
            
            indexOffset = 0;
            for i = 1:length(indexList)
                if (ischar(indexList{i}))
                    indexData = 1:obj.dataEntries.(elementNames{i}).numDimensions;
                else
                    indexData = indexList{i};
                end
                obj.setDataEntryInternal(elementNames{i}, indexList{i}, dataVector(:, indexOffset + (1:length(indexData))), varargin{:});
                indexOffset = length(indexData) + indexOffset;
            end
        end
        
        function [] = setDataEntryFlatIndex(obj, aliasName, dataVector, flatIndex)
            % Same as getDataEntryFlatIndex, just for setting.
            if (~isfield(obj.dataEntries, aliasName))
                error('Fieldname %s is not an entry or alias\n', aliasName);
            end
            
            if (size(dataVector, 2) ~= obj.dataEntries.(aliasName).numDimensions)
                error('Set data has wrong dimensionality for data entry %s (%d instead of %d)\n', aliasName, size(dataVector, 2), obj.dataEntries.(aliasName).numDimensions);
            end
            
            elementNames = obj.dataEntries.(aliasName).entryName;
            indexList = obj.dataEntries.(aliasName).indexList;
            
            indexOffset = 0;
            for i = 1:length(elementNames)
                if (ischar(indexList{i}))
                    indexData = 1:obj.dataEntries.(elementNames{i}).numDimensions;
                else
                    indexData = indexList{i};
                end
                dataVectorTemp = dataVector(:, indexOffset + indexData);
                obj.setDataEntryFlatIndexInternal(elementNames{i}, indexList{i}, dataVectorTemp, varargin{:});
                indexOffset = length(indexData) + indexOffset;
            end
        end
        
        function [data3DMatrix] = getDataEntry3D(obj, aliasName)
            % Instead of returning a concatened 2D matrix, we can also
            % obtain a 3D matrix fro the 2nd layer of the hierarchy.
            % However, this only works if ALL members of the 2nd layer have
            % the same number of elements.
            [elementName, indexList] = obj.checkDataAliasFeatures(aliasName);
            
%             assert(length(elementName) == 1, '3D Representation only implemented for single data aliases, not multiple ones!');
            
            % find the minimum length
            elementName_1 = elementName{1};
            depth = obj.dataManager.getDataEntryDepth(elementName_1);
            dataStruct = obj.getDataStructureForLayer(depth);
            minLength = min(cellfun(@length,{dataStruct(:).(elementName_1)}));
            

            data3DMatrix = [];
            for i = 1:length(elementName)
                elementName_i = elementName{i};
                indexList_i = indexList{i};

                depth = obj.dataManager.getDataEntryDepth(elementName_i);
                dataStruct = obj.getDataStructureForLayer(depth);
                trimmedData = cellfun(@(a) a(1:minLength,:),{dataStruct(:).(elementName_i)},'UniformOutput',false);
                data3DMatrix_i = cat(3, trimmedData{:});
                data3DMatrix_i = permute(data3DMatrix_i, [3 ,1, 2]);
                data3DMatrix = cat(3,data3DMatrix,data3DMatrix_i(:,:,indexList_i));
            end
        end
        
        function [] = setDataEntry3D(obj, aliasName, data3DMatrix)
            % Same as getDataEntry3D but for setting.
            if (~isfield(obj.dataEntries, aliasName))
                error('Fieldname %s is not an entry or alias\n', aliasName);
            end
            if (size(dataVector, 3) ~= obj.dataEntries.(aliasName).numDimensions)
                error('Set data has wrong dimensionality for data entry %s (%d instead of %d)\n', aliasName, size(dataVector, 3), obj.dataEntries.(aliasName).numDimensions);
            end
            
            elementName = obj.dataEntries.(aliasName).entryName;
            indexList = obj.dataEntries.(aliasName).indexList;
            
            assert(length(elementName) == 1, '3D Representation only implemented for single data aliases, not multiple ones!');
            
            elementName = elementName{1};
            indexList = indexList{1};
            
            depth = obj.dataManager.getDataEntryDepth(elementName);
            dataStructure = obj.getDataStructureForLayer(depth);
            
            for i = 1:length(dataStructure)
                dataStructure(i).(elementName)(:, indexList) = permute(data3DMatrix(i, :, :), [2 3 1]);
            end
            
            obj.setDataStructureForLayer(dataStructure, depth);
        end
        
        function [dataEntryCell] = getDataEntryCell(obj, aliasName)
            % Returns the data entry matrix for each member of the
            % hierarchy in an own cell. For example, if we have 50
            % episodes and we want to get the states for each episode, we
            % will get back a 50x1 cell array.
            [elementName, indexList] = obj.checkDataAliasFeatures(aliasName);
            
            assert(length(elementName) == 1, 'Cell Representation only implemented for single data aliases, not multiple ones!');
            
            elementName = elementName{1};
            indexList = indexList{1};
            
            depth = obj.dataManager.getDataEntryDepth(elementName);
            dataStructure = obj.getDataStructureForLayer(depth);
            dataEntryCell = {dataStructure(:).(elementName)};
            if (obj.dataEntries.(aliasName).numDimensions ~= obj.dataEntries.(elementName).numDimensions)
                dataEntryCell = cellfun( @(param_) param_(:, indexList), dataEntryCell);
            end
        end
        
        function [] = setDataEntryCell(obj, elementName, dataEntryCell)
            % Same as getDataEntryCell but for setting.
            dataEntryMatrix = cell2mat(dataEntryCell);
            obj.setDataEntry(elementName, dataEntryMatrix);
        end
        
        function [dataEntryCell] = getDataEntryCellArray(obj, elementNameList, varargin)
            % Similar to getDataEntry just that it works for several
            % dataEntries (or aliases). elementNameList is a cell array of
            % data entry/alias names. Returns a cell array returning the
            % results of the getDataEntry calls for each element of the
            % cell array.
            % Heavily used by the DataManipulation interface.
            dataEntryCell = {};
            for i = 1:length(elementNameList)
                if (~iscell(elementNameList{i}))
                    dataEntry = obj.getDataEntry(elementNameList{i}, varargin{:});
                else
                    %                     dataEntry = obj.getDataEntry(elementNameList{i}{1}, varargin{:});
                    dataEntry = [];
                    for j = 1:length(elementNameList{i})
                        dataEntry = [dataEntry, obj.getDataEntry(elementNameList{i}{j}, varargin{:})];
                    end
                end
                dataEntryCell{i} = dataEntry;
            end
        end
        
        function [dataEntryCell] = setDataEntryCellArray(obj, elementNameList, dataEntryCell, varargin)
            % Similar to setDataEntry just that it works for several
            % dataEntries (or aliases).
            % Heavily used by the DataManipulation interface.
            for i = 1:length(elementNameList)
                if (~iscell(elementNameList{i}))
                    obj.setDataEntry(elementNameList{i}, dataEntryCell{i}, varargin{:});
                else
                    index = 1;
                    for j = 1:length(elementNameList{i})
                        numDimensions = obj.dataEntries.(elementNameList{i}{j}).numDimensions;
                        dataMatrix = dataEntryCell{i}(:, index:(index + numDimensions-1));
                        obj.setDataEntry(elementNameList{i}, dataMatrix, varargin{:});
                        index = index + numDimensions;
                    end
                end
            end
        end
        
        function [numDimensions] = getNumDimensions(obj, elementName)
            % Returns the number of dimensions for a specific data
            % entry alias.
            numDimensions = obj.dataManager.getNumDimensions(elementName);
        end
        
        function [data] = getSubDataObject(obj, index)
            % Creates a new data object for the index-th element of the
            % lower hierarchy.
            subManager = obj.dataManager.getSubDataManager();
            
            data = Data.Data(subManager, obj.dataStructure.(subManager.getManagerName())(index));
        end
        
        function [newData, dataIndices] = createSubDataObjectsForSubIndex(obj, depth, index, dataEntries)
            % ... not sure whether this is ever useful...
            dataStructure = obj.getDataStructureForLayer(depth);
            if (~exist('dataEntries', 'var'))
                dataEntries = subManager.getElementNamesLocal();
            end
            
            dataManager = obj.dataManager.getDataManagerForDepth(depth);
            dataIndices = [];
            for j = 1:length(index)
                
                indexData = find([dataStructure(:).numElements] > index(j));
                dataStructureLocal = dataStructure(indexData);
                numElementList = [dataStructureLocal(:).numElements];
                numElementList = cumsum(numElementList);
                numElementList = [0 numElementList(1:end-1)];
                dataIndices = [dataIndices; numElementList + index(j)];
                
                newDataStruct(j) = dataManager.getDataStructure(length(dataStructureLocal));
                
                
                for k = 1:length(dataEntries)
                    dataMatrix = cat(1, dataStructureLocal.(dataEntries{k}));
                    newDataStruct(j).(dataEntries{k}) = dataMatrix(numElementList + index(j),:);
                end
                newDataObject(j) = Data.Data(dataManager, newDataStruct);
            end
        end
        
        function [] = resetFeatureTags(obj)
            % Resets the feature tags of all features in the data object.
            % This NEEDS to be done whenever we write new data into the
            % data structure, for example, when we sample new episodes.
            % Otherwise the feature generators would not realize that the
            % features need to be recomputed
            dataEntriesNames = fieldnames(obj.dataEntries);
            for i = 1:length(dataEntriesNames)
                if (obj.dataEntries.(dataEntriesNames{i}).isFeature)
                    numElements = obj.getNumElements(dataEntriesNames{i});
                    
                    obj.setDataEntry([dataEntriesNames{i}, 'Tag'], zeros(numElements, 1));
                    
                end
            end
        end
        
        function [newdata] = cloneDataSubSet(obj, subsetIndices)
            % Clones the data object. We can specify a subset for the
            % elements that we want to clone.
            datastruct = obj.dataManager.createNewDataStructureFromCopyIndex(obj.getDataStructure(), subsetIndices);
            newdata = Data.Data(obj.dataManager, datastruct);
        end
        
        function [] = printDataAliases(obj)
            % prints the names, dataEntries, numDimensions, Depth and
            % isFeature for each data alias defined for this data structure
            fieldNames = fieldnames(obj.dataEntries);
            for i = 1:length(fieldNames)
                fprintf('DataEntry: %s -> { ', fieldNames{i});
                entryNames = obj.dataEntries.(fieldNames{i}).entryNames;
                for j = 1:length(entryNames)
                    fprintf('%s ', entryNames{j});
                end
                fprintf('}, numDimensions: %d, Depth: %d, isFeature: %d\n', obj.dataEntries.(fieldNames{i}).numDimensions, obj.dataEntries.(fieldNames{i}).depth, obj.dataEntries.(fieldNames{i}).isFeature);
            end
        end
    end
    
    methods (Access = private)
        
        function [] = setDataEntryInternal(obj, elementName, indexListDimensions, dataVector, varargin)
            % Internal function for setting the data. Only touch if really
            % needed.
            if (obj.dataEntries.(elementName).restrictToRange)
                dataVector = bsxfun(@max, bsxfun(@min, dataVector, obj.dataEntries.(elementName).maxRange), obj.dataEntries.(elementName).minRange);
            end
            isSparse = obj.dataEntries.(elementName).isSparse;
            
            if (isempty(varargin))
                depth = obj.dataEntries.(elementName).depth;
                
                dataStructure = obj.getDataStructureForLayer(depth);
                numElementsList = [dataStructure(:).numElements];
                
                if (~isSparse)
                    dataMatrix = cat(1, dataStructure(:).(elementName));
                    dataMatrix(:, indexListDimensions) = dataVector;

                    dataEntryCell = mat2cell(dataMatrix, numElementsList, size(dataMatrix,2));
                    [dataStructure(:).(elementName)] = deal(dataEntryCell{:});
                    obj.setDataStructureForLayer(dataStructure, depth);
                else
                    dataMatrix = cat(2, dataStructure(:).(elementName));
                    dataMatrix(indexListDimensions, :) = dataVector';

                    dataEntryCell = mat2cell(dataMatrix, size(dataMatrix,1), numElementsList);
                    [dataStructure(:).(elementName)] = deal(dataEntryCell{:});
                    obj.setDataStructureForLayer(dataStructure, depth);
                end
            else
                layerIndex = varargin;
                switch obj.dataEntries.(elementName).depth
                    case 1
                        if (~isSparse)                            
                            obj.dataStructure.(elementName)(layerIndex{1}, indexListDimensions) = dataVector;                        
                        else                            
                            obj.dataStructure.(elementName)(indexListDimensions, layerIndex{1}) = dataVector';                        
                        end
                        
                    case 2
                        if (length(layerIndex) == 1)
                            layerIndex{2} = ':';
                        end
                        if (ischar(layerIndex{1}(1)) && layerIndex{1}(1) == ':')
                            if (ischar(layerIndex{2}(1)) && layerIndex{2}(1) == ':')
                                for i = 1:length(obj.dataStructure.(obj.dataManagerNames{2}))
                                    if (~isSparse)                            
                                        obj.dataStructure.(obj.dataManagerNames{2})(i).(elementName)(:, indexListDimensions) = dataVector;                                    
                                    else                                        
                                        obj.dataStructure.(obj.dataManagerNames{2})(i).(elementName)(indexListDimensions, :) = dataVector';
                                    end
                                end
                            else
                                for i = 1:length(obj.dataStructure.(obj.dataManagerNames{2}))
                                    if (~isSparse)
                                        obj.dataStructure.(obj.dataManagerNames{2})(i).(elementName)(layerIndex{2}, indexListDimensions) = dataVector(i,:);
                                    else                                        
                                        obj.dataStructure.(obj.dataManagerNames{2})(i).(elementName)(indexListDimensions, layerIndex{2}) = dataVector(i,:)';
                                    end
                                end
                            end
                        else
                            if (ischar(layerIndex{2}(1)) && layerIndex{2}(1) == ':')
                                index = 1;
                                for i = 1:length(layerIndex{1})                                    
                                    if (~isSparse)
                                        numData = size(obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(elementName), 1);
                                        obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(elementName)(:, indexListDimensions) = dataVector(index:(index + numData - 1), :);
                                    else
                                        numData = size(obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(elementName), 2);
                                        obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(elementName)(indexListDimensions, :) = dataVector(index:(index + numData - 1), :)';
                                    end
                                    index = numData + index;
                                end
                            else
                                dataTemp = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1});

                                
                                %is layerIndex{2} a column or row vector ?
                                if size(layerIndex{2},2)==1 
                                    if (size(layerIndex{2},1) > 1)
                                        layerIndex{2}=layerIndex{2}';
                                    else
                                        layerIndex{2}=repmat(layerIndex{2},1,size(dataVector,1));
                                    end
                                end
                                if all(size(layerIndex{1}) == 1)
                                    if (~isSparse)
                                        dataTemp.(elementName)(layerIndex{2},:) = dataVector;
                                    else
                                        dataTemp.(elementName)(:, layerIndex{2}) = dataVector';
                                    end
                                else
                                    for i=1:size(dataVector,1)
                                        element = dataTemp(i).(elementName);
                                        if (~isSparse)
                                            element(layerIndex{2}(i),:) = dataVector(i,:);
                                        else
                                            element(:, layerIndex{2}(i)) = dataVector(i,:);
                                        end
                                        dataTemp(i).(elementName) = element;
                                        
                                    end
                                end
                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}) = dataTemp;

                            end
                        end
                        
                    case 3
                        
                        
                        if (length(layerIndex) == 1)
                            layerIndex{2} = ':';
                        end
                        if (length(layerIndex) == 2)
                            layerIndex{3} = ':';
                        end
                        
                        if (ischar(layerIndex{1}(1)) && layerIndex{1}(1) == ':')
                            if (ischar(layerIndex{2}(1)) && layerIndex{2}(1) == ':')
                                if (ischar(layerIndex{3}(1)) && layerIndex{3}(1) == ':')
                                    for i = 1:length(obj.dataStructure.(obj.dataManagerNames{2}))
                                        for j = 1:length(obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3}))
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(j).(elementName)(:, indexListDimensions) = dataVector;
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(j).(elementName)(indexListDimensions, :) = dataVector';                                                
                                            end
                                        end
                                    end
                                else
                                    index = 1;
                                    for i = 1:length(obj.dataStructure.(obj.dataManagerNames{2}))
                                        for j = 1:length(obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3}))
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(j).(elementName)(layerIndex{3}, indexListDimensions) = dataVector(index, :);
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(j).(elementName)(indexListDimensions, layerIndex{3}) = dataVector(:, index);
                                            end
                                            index = index + 1;
                                        end
                                    end
                                end
                            else
                                if (ischar(layerIndex{3}(1)) && layerIndex{3}(1) == ':')
                                    for i = 1:length(obj.dataStructure.(obj.dataManagerNames{2}))
                                        for j = 1:length(layerIndex{2})
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(:, indexListDimensions) = dataVector;
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(indexListDimensions, :) = dataVector';
                                            end
                                        end
                                    end
                                else
                                    index = 1;
                                    for i = 1:length(obj.dataStructure.(obj.dataManagerNames{2}))
                                        for j = 1:length(layerIndex{2})
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(layerIndex{3}, indexListDimensions) = dataVector(index, :);
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(indexListDimensions, layerIndex{3}) = dataVector(index, :)';
                                            end
                                            index = index + 1;
                                        end
                                    end
                                end
                            end
                        else
                            if (ischar(layerIndex{2}(1)) && layerIndex{2}(1) == ':')
                                if (ischar(layerIndex{3}(1)) && layerIndex{3}(1) == ':')
                                    for i = 1:length(layerIndex{1})
                                        for j = 1:length(obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3}))
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(j).(elementName)(:, indexListDimensions) = dataVector;
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(j).(elementName)(indexListDimensions, :) = dataVector';
                                            end
                                        end
                                    end
                                else
                                    index = 1;
                                    for i = 1:length(layerIndex{1})
                                        for j = 1:length(obj.dataStructure.(obj.dataManagerNames{2})(i).(obj.dataManagerNames{3}))
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(j).(elementName)(layerIndex{3}, indexListDimensions) = dataVector(index, :);
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(j).(elementName)(indexListDimensions, layerIndex{3}) = dataVector(index, :)';
                                            end
                                            index = index + 1;
                                        end
                                    end
                                end
                            else
                                if (ischar(layerIndex{3}(1)) && layerIndex{3}(1) == ':')
                                    for i = 1:length(layerIndex{1})
                                        for j = 1:length(layerIndex{2})
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(:,indexListDimensions) = dataVector;
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(indexListDimensions, :) = dataVector';
                                            end
                                        end
                                    end
                                else
                                    index = 1;
                                    for i = 1:length(layerIndex{1})
                                        for j = 1:length(layerIndex{2})
                                            if (~isSparse)
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(layerIndex{3}, indexListDimensions) = dataVector(index, :);
                                            else
                                                obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}(i)).(obj.dataManagerNames{3})(layerIndex{2}(j)).(elementName)(indexListDimensions, layerIndex{3}) = dataVector(index, :)';
                                            end
                                            index = index + 1;
                                        end
                                    end
                                end
                            end
                        end
                        
                        
                    otherwise
                        assert(false, sprintf('Level %d is not supported in data hierarchy', depth));
                end
            end            
            
        end
        
        function [] = setDataEntryFlatIndexInternal(obj, elementName, indexList, dataVector, flatIndex)
            % Internal function for setting the data. Only touch if really
            % needed.
            if (obj.dataEntries.(elementName).restrictToRange)
                dataVector = bsxfun(@max, bsxfun(@min, dataVector, obj.dataEntries.(elementName).maxRange), obj.dataEntries.(elementName).minRange);
            end
            
            depth = obj.dataManager.getDataEntryDepth(elementName);
            dataStructure = obj.getDataStructureForLayer(depth);
            dataMatrix = cat(1, dataStructure(:).(elementName));
            
            dataMatrix(flatIndex, indexList) = dataVector;
            obj.setDataEntry(elementName, dataMatrix);
        end
        
        function [dataMatrix] = getDataEntryInternal(obj, queryStructure)
            % Internal function for getting the data. Only touch if really
            % needed.
            %queryStructure = obj.dataManager.checkQueryString(elementName);
            elementName = queryStructure.elementName;
            indexList = queryStructure.indexList;
            layerIndex = queryStructure.layerIndex;
            if (isempty(layerIndex))
                depth = obj.dataEntries.(elementName).depth;
                dataStructure = obj.getDataStructureForLayer(depth);
                if (~queryStructure.isSparse)
                    dataMatrix = cat(1, dataStructure(:).(elementName));                
                else
                    dataMatrix = cat(2, dataStructure(:).(elementName))';                
                end
            else

                if (length(layerIndex) < obj.dataEntries.(elementName).depth)
                    [layerIndex{end +1:obj.dataEntries.(elementName).depth}] = deal(':');
                end
                
                switch obj.dataEntries.(elementName).depth
                    case 1
                        if (~queryStructure.isSparse)
                            dataMatrix = obj.dataStructure.(elementName)(layerIndex{1},:);
                        else
                            dataMatrix = obj.dataStructure.(elementName)(:, layerIndex{1})';
                        end
                        
                    case 2
                        
                        if (length(layerIndex{1}) > 1 || (ischar(layerIndex{1}) && layerIndex{1} == ':'))
                            %temp = arrayfun(@(elem_) elem_.(elementName)(layerIndex{2},:), obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}), 'UniformOutput', false);
                            %dataMatrix = cell2mat(temp);
                            
                            if (ischar(layerIndex{2}))
                                dataTemp = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1});
                                if (~queryStructure.isSparse)
                                    dataMatrix = cat(1, dataTemp(:).(elementName));
                                else
                                    dataMatrix = cat(2, dataTemp(:).(elementName))';
                                end
                            else
                                dataTemp = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1});
                                elementList = cumsum([dataTemp.numElements]);
                                elementList = [0 elementList(1:end-1)];
                                if (length(layerIndex{2}) > 1)
                                    elementList = repmat(elementList', 1, length(layerIndex{2}));
                                    elementList = bsxfun(@plus, elementList, layerIndex{2})';
                                    elementList = elementList(:);
                                else
                                    if (layerIndex{2} < 0)
                                        elementList = cumsum([dataTemp.numElements]) + layerIndex{2} + 1;
                                    else
                                        elementList = elementList + layerIndex{2};
                                    end
                                end
                                
                                %NOTE: This is only faster for large
                                %matrices, but the check takes too long and
                                %for small its still fast enough
                                %if obj.getNumElements(elementName)*obj.getNumDimensions(elementName)>100000
                                if (~queryStructure.isSparse)
                                    i=1;
                                    skipped=0;
                                    dataMatrix = zeros(length(elementList), size(dataTemp(1).(elementName),2));
                                    for j = 1:length(elementList)
                                        idx = elementList(j); 
                                        curElement = dataTemp(i).(elementName);
                                        lgt = size(curElement,1);
                                        while idx-skipped>lgt
                                            i=i+1;
                                            skipped=skipped+lgt;
                                            curElement = dataTemp(i).(elementName);
                                            lgt = size(curElement,1);
                                        end
                                        if size(dataMatrix,2)==0
                                            dataMatrix = zeros(size(elementList,2),size(curElement(idx-skipped,:),2));
                                        end
                                        dataMatrix(j,:) = curElement(idx-skipped,:);
                                    end
                                else
                                    i=1;
                                    skipped=0;
                                    dataMatrix = spalloc(size(dataTemp(1).(elementName),1), length(elementList), length(elementList));
                                    for j = 1:length(elementList)
                                        idx = elementList(j); 
                                        curElement = dataTemp(i).(elementName);
                                        lgt = size(curElement,2);
                                        while idx-skipped>lgt
                                            i=i+1;
                                            skipped=skipped+lgt;
                                            curElement = dataTemp(i).(elementName);
                                            lgt = size(curElement,2);
                                        end
                                        if size(dataMatrix,2)==0
                                            dataMatrix = spalloc(size(curElement, 1), length(elementList), length(elementList));
                                        end
                                        dataMatrix(:, j) = curElement(:, idx-skipped);
                                    end
                                    dataMatrix = dataMatrix';
                                end
                                %else
                                %    dataMatrixTmp = cat(1, dataTemp(:).(elementName));
                                %  dataMatrix = dataMatrixTmp(elementList, :);
                                %end
                            end
                        else
                            if (~queryStructure.isSparse)
                                if (layerIndex{2} < 0)
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(elementName)(end + layerIndex{2} + 1, :);
                                else
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(elementName)(layerIndex{2}, :);
                                end
                            else
                                if (layerIndex{2} < 0)
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(elementName)(:, end + layerIndex{2} + 1)';
                                else
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(elementName)(:, layerIndex{2})';
                                end

                            end
                        end
                        
                    case 3
                        if (length(layerIndex{2}) > 1  || length(layerIndex{1}) > 1 || (ischar(layerIndex{1}) && layerIndex{1} == ':') || (ischar(layerIndex{2}) &&  layerIndex{2} == ':'))
                            dataStructure = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1});
                            if (length(layerIndex{2}) == 1 && layerIndex{2} < 0)
                                dataStructure = cell2mat(arrayfun(@(elem_) elem_.(obj.dataManagerNames{3})(end + layerIndex{2} + 1), dataStructure, 'UniformOutput', false));
                            else
                                dataStructure = cell2mat(arrayfun(@(elem_) elem_.(obj.dataManagerNames{3})(layerIndex{2}), dataStructure, 'UniformOutput', false));
                            end
                            
                            if (~queryStructure.isSparse) 
                                if (length(layerIndex{3}) == 1 && layerIndex{3} < 0)
                                    dataMatrix = cell2mat(arrayfun(@(elem_) elem_.(elementName)(end + layerIndex{3} + 1,:), dataStructure, 'UniformOutput', false));
                                else
                                    dataMatrix = cell2mat(arrayfun(@(elem_) elem_.(elementName)(layerIndex{3},:), dataStructure, 'UniformOutput', false));
                                end
                            else
                                if (length(layerIndex{3}) == 1 && layerIndex{3} < 0)
                                    dataMatrix = cell2mat(arrayfun(@(elem_) elem_.(elementName)(:, end + layerIndex{3} + 1)', dataStructure, 'UniformOutput', false));
                                else
                                    dataMatrix = cell2mat(arrayfun(@(elem_) elem_.(elementName)(:, layerIndex{3})', dataStructure, 'UniformOutput', false));
                                end
                            end
                        else
                            endLayer3 = length(layerIndex{3}) == 1 && layerIndex{3} < 0;
                            if (~queryStructure.isSparse) 
                                if (endLayer3)
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(obj.dataManagerNames{3})(layerIndex{2}).(elementName)(end + layerIndex{3} + 1, :);
                                else
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(obj.dataManagerNames{3})(layerIndex{2}).(elementName)(layerIndex{3}, :);
                                end
                            else
                                if (endLayer3)
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(obj.dataManagerNames{3})(layerIndex{2}).(elementName)(:, end + layerIndex{3} + 1)';
                                else
                                    dataMatrix = obj.dataStructure.(obj.dataManagerNames{2})(layerIndex{1}).(obj.dataManagerNames{3})(layerIndex{2}).(elementName)(:, layerIndex{3});
                                end
                            end
                        end
                        
                    otherwise
                        assert(false, sprintf('Level %d is not supported in data hierarchy', depth));
                end
            end
            dataMatrix = dataMatrix(:, indexList);
            
            if (queryStructure.restrictRange)
                minRange = obj.dataEntries.(elementName).minRange(indexList);
                maxRange = obj.dataEntries.(elementName).maxRange(indexList);

                dataMatrix = bsxfun(@min, bsxfun(@max, dataMatrix, minRange), maxRange);
            end
        end
        
        function [dataMatrix] = getDataEntryFlatIndexInternal(obj, elementName, indexList, flatIndex)
            depth = obj.dataEntries.(elementName).depth;
            dataStructure = obj.getDataStructureForLayer(depth);
            dataMatrix = cat(1, dataStructure(:).(elementName));
            
            dataMatrix = dataMatrix(flatIndex,indexList);
        end
        
        
        function [elementNames, indexList, queryStructureArray] = checkDataAliasFeatures(obj, aliasName, varargin)
            useOldFeatures = false;
            if (length(aliasName) > 8 && strcmp(aliasName(end - 7:end), 'NoUpdate'))
                useOldFeatures = true;
                aliasName = aliasName(1:end-8);
            end
            
            queryStructure = obj.dataManager.checkQueryString(aliasName, varargin{:});
            aliasName = queryStructure.aliasName;
            
            if (~isfield(obj.dataEntries, aliasName))
                error('Fieldname %s is not an entry or alias\n', aliasName);
            end
                                 
            elementNames = obj.dataEntries.(aliasName).entryNames;
            indexList = obj.dataEntries.(aliasName).indexList;
            queryStructureArray = [];
            for i = 1:length(elementNames)
                if (~useOldFeatures && obj.dataEntries.(elementNames{i}).isFeature)
                    featureGenerator = obj.dataManager.getFeatureGenerator(elementNames{i});
                    featureGenerator.callDataFunction('generateFeatures', obj, varargin{:});
                end
                queryStructureArray(i).restrictRange = queryStructure.restrictRange;
                queryStructureArray(i).layerIndex = queryStructure.layerIndex;                
                queryStructureArray(i).elementName = elementNames{i};
                queryStructureArray(i).isSparse = obj.dataManager.isSparse(queryStructureArray(i).elementName);
                
                if (numel(indexList) >= i)
                    queryStructureArray(i).indexList = indexList{i};
                else
                    queryStructureArray(i).indexList = [];
                end
            end
        end
    end
    
    methods (Access = protected)
        function [obj] = initDataStructureEntries(obj)
            dataEntriesManager = obj.dataManager.getAliasNames();
            for i = 1:length(dataEntriesManager)
                obj.dataEntries.(dataEntriesManager{i}).depth = obj.dataManager.getDataEntryDepth(dataEntriesManager{i});
                tempStructure = obj.dataManager.getAliasStructure(dataEntriesManager{i});
                
                obj.dataEntries.(dataEntriesManager{i}).numDimensions = tempStructure.numDimensions;
                obj.dataEntries.(dataEntriesManager{i}).entryNames = tempStructure.entryNames;
                obj.dataEntries.(dataEntriesManager{i}).indexList = tempStructure.indexList;
                if (~isempty(tempStructure.entryNames))
                    obj.dataEntries.(dataEntriesManager{i}).restrictToRange = obj.dataManager.getRestrictToRange(tempStructure.entryNames{1});
                    obj.dataEntries.(dataEntriesManager{i}).isFeature = obj.dataManager.isFeature(dataEntriesManager{i});
                    obj.dataEntries.(dataEntriesManager{i}).isSparse = obj.dataManager.isSparse(dataEntriesManager{i});
                    
                else
                    obj.dataEntries.(dataEntriesManager{i}).restrictToRange = false;
                    obj.dataEntries.(dataEntriesManager{i}).isFeature = false;
                end
                obj.dataEntries.(dataEntriesManager{i}).maxRange = obj.dataManager.getMaxRange(dataEntriesManager{i});
                obj.dataEntries.(dataEntriesManager{i}).minRange = obj.dataManager.getMinRange(dataEntriesManager{i});
                
%                 if (~isempty(tempStructure.entryNames))
%                     obj.dataEntries.(dataEntriesManager{i}).restrictToRange = obj.dataManager.getRestrictToRange(tempStructure.entryNames{1});
%                 else
%                     obj.dataEntries.(dataEntriesManager{i}).restrictToRange = false;
%                 end
            end
            dataManager = obj.dataManager;
            obj.dataManagerNames = {};
            while (~isempty(dataManager))
                obj.dataManagerNames{end + 1} = dataManager.getManagerName();
                dataManager = dataManager.getSubDataManager();
            end
        end
    end
    
end