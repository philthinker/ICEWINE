classdef DMPBasisGenerator < TrajectoryGenerators.BasisFunctions.BasisGenerator
    
    properties
        centers;
        bandWidth;
        
    end
    
    properties (SetObservable, AbortSet)
        dt
        basisBandWidthFactor = 3
        basisEndTime = 1.0

    end
        
    methods
        
        function obj = DMPBasisGenerator(dataManager, phaseGenerator)
            
            obj = obj@TrajectoryGenerators.BasisFunctions.BasisGenerator(dataManager, phaseGenerator);
            
            obj.linkProperty('dt');
            obj.linkProperty('basisBandWidthFactor');
            obj.linkProperty('basisEndTime');
            
            obj.initializeBasisFunctions();
        end
        
        function [] = initializeBasisFunctions(obj)
            
            timePoints = linspace(obj.dt, obj.dt * obj.phaseGenerator.numTimeSteps * obj.basisEndTime, obj.numBasis);
            obj.centers = obj.phaseGenerator.generatePhaseFromTime(timePoints);

            tmpBandWidth = [obj.centers(2:end)-obj.centers(1:end-1), obj.centers(end)-obj.centers(end-1)];
            
            %The Centers should not overlap too much (makes w almost
            %random due to aliasing effect). Empirically chosen
            obj.bandWidth = obj.basisBandWidthFactor./(tmpBandWidth).^2;
            
        end
        
        function [basis] = generateBasis(obj, phase)
            
            tmpDiff = bsxfun(@minus,phase,obj.centers).^2;
            basis = exp(bsxfun(@times,tmpDiff, -obj.bandWidth / 2));
            
            basis = bsxfun(@rdivide, bsxfun(@times,basis,phase), sum(basis,2));
        end                
    end
end
