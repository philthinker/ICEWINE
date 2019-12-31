classdef Learner < DataPreprocessors.DataPreprocessor
    % The Learner.Learner class serves as a base class for all learners and predefines all necessary methods.
    
    properties
        dataPreprocessors
        featureGeneratorLearners
        
        dataNameLearner = 'data';
        
    end
    
    % Class methods
    methods
        function obj = Learner(varargin)
            obj = obj@DataPreprocessors.DataPreprocessor(varargin{:});
        end
        
        function [] = setDataNameLearner(obj, dataName)
            obj.dataNameLearner = dataName;
        end
        
        function obj = addedData(obj, data, newSampleIndices)
            
        end
        
        function obj = deletedData(obj, data, keepIndices)
            
        end
        
        function obj = updateModel(obj, data)
        end
        
        function obj = updateModelCollection(obj, dataCollection)
            obj.preprocessData(dataCollection);
            obj.updateModel(dataCollection.getDataObjectForName(obj.dataNameLearner));
        end
        
        
        function [] = printMessage(obj, data)
            
        end
        
        function [] = addDefaultCriteria(obj, trial, evaluationCriterium)
            
        end
        
        function preprocessDataCollection(obj, dataCollection)
            for i = 1:length(obj.dataPreprocessors)
                obj.dataPreprocessors{i}.setIteration(obj.iteration);
                obj.dataPreprocessors{i}.preprocessDataCollection(dataCollection);
            end
        end
        
        function [] = addDataPreprocessor(obj, dataPreprocessor)
            if (isempty(obj.dataPreprocessors))
                obj.dataPreprocessors{1} = dataPreprocessor;
            else
                obj.dataPreprocessors{end + 1} = dataPreprocessor;
            end
        end
        
        function data = learnFeatureGenerators(obj, data)
            for i = 1:length(obj.featureGeneratorLearners)
                obj.featureGeneratorLearners{i}.updateModel(data);
            end
        end
        
        function addFeatureGeneratorLearner(obj, featureGeneratorLearner)
            if (isempty(obj.featureGeneratorLearners))
                obj.featureGeneratorLearners{1} = featureGeneratorLearner;
            else
                obj.featureGeneratorLearners{end + 1} = featureGeneratorLearner;
            end
        end
    end
end
