classdef LinearTrajectoryGenerator < TrajectoryGenerators.TrajectoryGenerator
    
    properties(AbortSet, SetObservable)
        numBasis = 5;
        phaseGenerator
        basisGenerator
    end
    
    properties
       
    end
    
    methods
              
        function obj = LinearTrajectoryGenerator(dataManager, numJoints, phaseGenerator, basisGenerator, weightsMultiplier)            
            obj = obj@TrajectoryGenerators.TrajectoryGenerator(dataManager, numJoints);  
            
            if (~exist('weightsMultiplier', 'var'))
                weightsMultiplier = 1.0;
            end
            
            obj.linkProperty('numBasis');
            
            if (~exist('phaseGenerator', 'var') || isempty(phaseGenerator))
                obj.phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);
            else
                obj.phaseGenerator = phaseGenerator;
            end

            if (~exist('basisGenerator', 'var') || isempty(basisGenerator))
                obj.basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedGaussianBasisGenerator(dataManager,obj.phaseGenerator);
            else
                obj.basisGenerator = basisGenerator;
            end
                                   
            dimWeights = obj.numJoints * obj.numBasis;
            maxWeights = 5 * ones(1, dimWeights) * weightsMultiplier;
            
            level = dataManager.getDataManagerDepth('steps') - 1;
            obj.registerOptionalParameter('Weights', true, dimWeights, -maxWeights, maxWeights, 'parameters', level);
            obj.Weights = zeros(obj.numBasis * obj.numJoints, 1);
            
               
            obj.registerTrajectoryFunction();
        end    
        
        function [additionalParameters] = getParameterNamesForTrajectoryGenerator(obj)
            additionalParameters = {obj.additionalParameters{:}, obj.phaseGenerator.additionalParameters{:}};
        end                

        function [weights] = getWeights(obj)
            weights = reshape ( obj.Weights, obj.numBasis, []);
        end
        
        function [referencePos, referenceVel, referenceAcc] = getReferenceTrajectory(obj, basis, varargin)
            obj.inputParameterDeMux(varargin);

            w = reshape ( obj.Weights, obj.numBasis, [] );
            referencePos = basis * w;
            referenceVel = diff(referencePos); 
            referenceVel = [referenceVel; referenceVel(end,:)];                     
            referenceAcc = diff(referenceVel);            
            referenceAcc = [referenceAcc; referenceAcc(end,:)];
        end
        
        function [] = registerTrajectoryFunction(obj)
            obj.addDataManipulationFunction('getReferenceTrajectory', [{'basis'} ,obj.additionalParameters{:}], {'referencePos', 'referenceVel', 'referenceAcc'}, Data.DataFunctionType.PER_EPISODE);
        end
        
        function [] = registerAdditionalParametersInData(obj, data, suffix, index)
            obj.registerAdditionalParametersInData@TrajectoryGenerators.TrajectoryGenerator(data, suffix, index);
            obj.phaseGenerator.registerAdditionalParametersInData(data, suffix, index);
        end
        
        function [] = disableParametersFromData(obj)
            obj.disableParametersFromData@TrajectoryGenerators.TrajectoryGenerator();
            obj.phaseGenerator.disableParametersFromData();
        end
        
        function [] = enableParametersFromData(obj)
            obj.enableParametersFromData@TrajectoryGenerators.TrajectoryGenerator();
            obj.phaseGenerator.enableParametersFromData();
        end

                                                                       
    end        
                  
end