classdef BasisGenerator < FeatureGenerators.FeatureGenerator
    
    properties (SetObservable, AbortSet)
        numBasis = 20;
    end
    
    properties
        phaseGenerator;
    end
    
    methods
        
        function obj = BasisGenerator(dataManager, phaseGenerator, basisName)
            if (~exist('basisName', 'var'))
                basisName = 'basis';
            end
            
            obj = obj@FeatureGenerators.FeatureGenerator(dataManager, {phaseGenerator.outputName},['~' basisName]);
            
            obj.phaseGenerator = phaseGenerator;
            obj.linkProperty('numBasis');
            obj.setNumFeatures(obj.numBasis);
            
            obj.setIsPerEpisodeFeatureGenerator();
            %dataManager.addDataEntry('steps.basis', obj.numBasis );
            obj.addDataManipulationFunction('generateBasis', {phaseGenerator.outputName}, {basisName}, Data.DataFunctionType.PER_EPISODE );
        end
        
        function [features] = getFeaturesInternal(obj, numElements, varargin)
            features = obj.generateBasis(varargin{:});
        end
    end
    
    methods (Abstract)
        [basis] = generateBasis(obj, phase);
    end
end
