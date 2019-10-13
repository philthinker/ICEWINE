classdef FeatureGenerator < Data.DataManipulator
    % The FeatureGenerator is a Data.DataManipulator that handles Features. 
    %
    % Features are DataEntries that have to be recalculated at certain points. 
    % The do this need to specify a set of data entries in the constructor 
    % and create a feature tag vector as a new data entry in the dataManager
    % called by the composed string <tt> <name of the dataEntry> + <featureName> + ‘Tag’</tt>.
    % When some of the specified data entries are outdated, you need to set
    % the corresponding entry in the feature tag vector to -1. Whenever 
    % <tt>Data.getData</tt> gets called for this specified data entry every
    % sub entry whose tag is -1 will be recalculated.
    %
    % In the constructor you need to specify which data entries of the data 
    % manager will be overwatched by the feature generator by giving the
    % name and the subset of those entries. You also need to specify a feature 
    % name of the feature generator. 
    %
    % The function <tt>generateFeatures()</tt> be handled by the a subclass
    % of Datamanagaer that can handle Feature generators (for example
    % <tt>FunctionLinearInFeatures</tt>) when <tt>getData()</tt> is called. 
    %
    % To create your own <tt>FeatureGenerator</tt> you need to define the 
    % abstract function <tt>getFeaturesInternal</tt>. This function defines
    % the mapping from the input data entries to the feature entry.
    %
    % The parameter of the abstract function should be numElements as the 
    % number of input elements and varargin that should contain the input
    % data and optional parameters of your mapping.

   properties
        
        
        featureName = ''; % Name of the feature in this generator
        
        featureVariables % dataentry 
        outputName
        
        stateIndices = [];
        dimSample
        numFeatures = 0;
        
        managerForFeatureLayer; % datamanager that handles the features
        
        layer;
        
        layerForInputVariable;
        isPerEpisodeFeature = false;
        isPerEpisodeCallFunction = false;
        
        isSparse = false;
    end
    
    methods
        function [obj] = FeatureGenerator(dataManager, featureVariables, featureName, stateIndices, numFeatures, isPerEpisode)
            % @param dataManager DataManger to operate on
            % @param featureVariables Set of dataentries  in the DataManager as strings
            % @param featureName determines the name of the features (for more information see the function description)
            % @param stateIndices subset of the dataentries this feature generator will handle
            % @param numFeatures Dimension of the features
            % @param isPerEpisode flag if the feature generator should update every episode
            % The constructor will create two new data entries in the given
            % dataManager. The first are the feature entry named with the 
            % composite string  <tt> <name of the dataEntry> + <featureName> </tt>
            % or only  <tt> <name of the dataEntry></tt> if the feature 
            % name is <tt>’~’</tt>. The second one is the feature tag vector
            % named <tt><name of feature entry>+ ‘Tag’</tt>.
            obj = obj@Data.DataManipulator(dataManager);
            obj.featureName = featureName;
            if (~iscell(featureVariables))
                featureVariables = {featureVariables};
            end
            
            if (~exist('isPerEpisode', 'var'))
                obj.isPerEpisodeFeature = false;
            else
                obj.isPerEpisodeFeature = isPerEpisode;
            end
            obj.isPerEpisodeCallFunction = obj.isPerEpisodeFeature;
            
            if (featureName(1) ~= '~' && all(obj.dataManager.isDataAlias(featureVariables{1})))
                if (iscell(featureVariables{1}))
                    obj.outputName = [featureVariables{1}{:}, featureName];
                else
                    obj.outputName = [featureVariables{1}, featureName];
                end
            else
                obj.outputName = featureName(2:end);
            end
            
            assert(~isempty(featureVariables), 'Feature Variables must either contain data entries or the data manager name if no feature variables are used as input')
            if (obj.isPerEpisodeFeature)
                obj.managerForFeatureLayer = obj.dataManager.getDataManagerForName('episodes');
            else
                if (~obj.dataManager.isDataAlias(featureVariables{1}))
                    obj.managerForFeatureLayer = obj.dataManager.getDataManagerForName(featureVariables{1});
                    featureVariables = featureVariables(2:end);
                else
                    obj.managerForFeatureLayer = obj.dataManager.getDataManagerForEntry(featureVariables{1});
                end
            end
            assert(~isempty(obj.managerForFeatureLayer), 'Feature Variables must either contain data entries or the data manager name if no feature variables are used as input')
            obj.layer = obj.dataManager.getDataManagerDepth(obj.managerForFeatureLayer.getManagerName());
            
            obj.setFeatureInputArguments(featureVariables{:});
            
            if ( exist('numFeatures', 'var') && ~isempty(numFeatures) )
                obj.numFeatures = numFeatures;
            end
            
            if ( exist('stateIndices', 'var') && ~isempty(stateIndices) )
                obj.setStateIndices(stateIndices);
            else
                obj.setStateIndices(':');
            end

            if (~exist('isPerEpisodeFeatureGenerator', 'var'))
                isPerEpisodeFeatureGenerator = false;
            end
            
            if (isPerEpisodeFeatureGenerator)
                obj.setIsPerEpisodeFeatureGenerator();
            end
            
        end
        
        function [isSparse] = getIsSparse(obj)
            isSparse = obj.isSparse;
        end
        
        function [isSparse] = setIsSparse(obj, isSparse)
            obj.isSparse = isSparse;
            if (obj.isSparse)
                obj.managerForFeatureLayer.setSparse(obj.outputName, true);
            end
        end
        
        
        function [] = setIsPerEpisodeCallFunction(obj)
            obj.isPerEpisodeCallFunction = true;
            obj.setFeatureInputArguments();
        end
        
        function [] = setIsPerEpisodeFeatureGenerator(obj)
            obj.isPerEpisodeFeature = true;
            obj.isPerEpisodeCallFunction = true;
            obj.setCallType('getFeatures', Data.DataFunctionType.PER_EPISODE);
            obj.setCallType('generateFeatures', Data.DataFunctionType.PER_EPISODE);
        end
        
        function [] = initObject(obj)
            manager = obj.dataManager.getDataManagerForEntry(obj.featureVariables{1});
            manager.addDataEntry(obj.outputName, obj.getNumFeatures());
            manager.setFeatureGenerator(obj.outputName, obj);
        end
        
        function [output] = getFeatureName(obj)
            output = obj.outputName;
        end
        
        function [] = registerFeatureInData(obj)
            obj.managerForFeatureLayer.addDataEntry(obj.outputName, obj.getNumFeatures());
            obj.managerForFeatureLayer.setFeatureGenerator(obj.outputName, obj);
            if (obj.isSparse)
                obj.managerForFeatureLayer.setIsSparse(obj.outputName, true);
            end
            obj.dataManager.finalizeDataManager();
            
        end
        
        function [] = setNumFeatures(obj, numFeatures)
            obj.numFeatures = numFeatures;
            
            obj.registerFeatureInData();
        end
        
        function [obj] = setStateIndices(obj, indices)
            
            if (islogical(indices))
                indices = find(indices);
            end
            
            if (length(indices) == 1 && ischar(indices) && indices(1) == ':')
                if (iscell(obj.featureVariables))
                    obj.stateIndices = 1:obj.dataManager.getNumDimensions(obj.featureVariables{1});
                else
                    obj.stateIndices = 1:obj.dataManager.getNumDimensions(obj.featureVariables);
                end
            else
                obj.stateIndices = indices;
            end
            %obj.stateIndices = indices;
            
            obj.dimSample = size(obj.stateIndices, 2);
            
            obj.registerFeatureInData();
            %obj.setInputIndices('getFeatures', 1, obj.stateIndices);
        end
        
        function [] = setFeatureInputArguments(obj, varargin)
            if (~isempty(varargin))
                localFeatureVariables = varargin;
                if (~iscell(localFeatureVariables))
                    localFeatureVariables = {{localFeatureVariables}};
                elseif(~iscell(localFeatureVariables{1}))
                    localFeatureVariables = cellfun(@(x) {x}, localFeatureVariables, 'UniformOutput', false);
                end
                obj.featureVariables = localFeatureVariables;
            end
            obj.addDataManipulationFunction('getFeatures', obj.featureVariables, obj.outputName, true, true);
            
            if (~isempty(obj.featureVariables))
                obj.addDataManipulationFunction('generateFeatures', {[obj.outputName, 'NoUpdate'], [obj.outputName, 'Tag'], obj.featureVariables{:}}, {obj.outputName, [obj.outputName, 'Tag']}, true, true);
            else
                obj.addDataManipulationFunction('generateFeatures', {[obj.outputName, 'NoUpdate'], [obj.outputName, 'Tag']}, {obj.outputName, [obj.outputName, 'Tag']}, true, true);
            end
            
            obj.layerForInputVariable = zeros(length(obj.featureVariables),1);
            for i = 1:length(obj.layerForInputVariable)
                obj.layerForInputVariable(i) = obj.dataManager.getDataEntryDepth(obj.featureVariables{i});
            end
            
            if (obj.isPerEpisodeCallFunction)
                obj.setCallType('getFeatures', Data.DataFunctionType.PER_EPISODE);
                obj.setCallType('generateFeatures', Data.DataFunctionType.PER_EPISODE);
            end
            
        end
        
        function [features] = getFeatures(obj, numElements, inputMatrix, varargin)
            if (nargin >= 3)
                features = obj.getFeaturesInternal(numElements, inputMatrix(:, obj.stateIndices), varargin{:});
            else
                features = obj.getFeaturesInternal(numElements);
            end
        end
        
        function [features, featureTags] = generateFeatures(obj, numElements, oldFeatureVector, oldFeatureTags, varargin)
            % @param numElements
            % @param oldFeatureVector data entries that will be checked if they need recalculating 
            % @param oldFeatureTags data entries will be updated if the corresponding feature tag is not 1. It will be 0 if its un initiated and -1 of its in need of recalculating. 
            % @param varargin set of new caluclated features
            % The <tt>generateFeatures</tt> function will be called by 
            % <tt>Data.getDataEntry</tt>. It determines if any of the entries 
            % of the <tt>oldFeaturesVector</tt> have to be recalculated by 
            % checking the <tt>oldFeatureTags</tt>. Every data entry in the
            % feature vector that has not 1 as its corresponding feature 
            % tag will be calculated anew. 
            isValid = obj.isValidFeatureTag(oldFeatureTags);
            
            features = oldFeatureVector;
            featureTags = oldFeatureTags;
            
            if (~all(isValid))
                inputArgs = varargin;
                for i = 1:length(inputArgs)
                    if (obj.layer == obj.layerForInputVariable(i))
                        inputArgs{i} = inputArgs{i}(~isValid,:);
                    end
                end
                % overwhite old features where appropriate 
                featuresNew = obj.getFeatures(sum(~isValid), inputArgs{:});
                features(~isValid,:) = featuresNew;
                featureTags(~isValid) = obj.getFeatureTag();
            end
        end
        
        function [featureTag] = getFeatureTag(obj)
            featureTag = 1;
        end
        
        function [isValid] = isValidFeatureTag(obj, featureTags)
            isValid = featureTags > 0;
        end
        
        function [numFeatures] = getNumFeatures(obj)
            numFeatures = obj.numFeatures;
        end
    end
    
    methods (Abstract)
        % This function defines the mapping from the input data entries to
        % the feature entry.
        %
        % The parameter of the abstract function should be numElements as the 
        % number of input elements and varargin that should contain the input
        % data and optional parameters of your mapping.
        [features] = getFeaturesInternal(obj, numElements, varargin);
    end
end


