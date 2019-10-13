classdef BasisGeneratorDerivatives < TrajectoryGenerators.BasisFunctions.BasisGenerator
    
    methods
        
        function obj = BasisGeneratorDerivatives(dataManager,phaseGenerator,basisName)
            if (~exist('basisName', 'var'))
                basisName = 'basis';
            end
            
            obj = obj@TrajectoryGenerators.BasisFunctions.BasisGenerator(dataManager,phaseGenerator,basisName);
            
            dataManager.addDataEntry( ['steps.' basisName 'D'],  obj.numBasis );
            dataManager.addDataEntry( ['steps.' basisName 'DD'], obj.numBasis );
            
            % For computing the derivatives \frac{ \par Basis }{ \par phase}
            obj.addDataManipulationFunction('generateBasisD', {phaseGenerator.outputName}, {[basisName 'D']}, Data.DataFunctionType.PER_EPISODE );
            obj.addDataManipulationFunction('generateBasisDD', {phaseGenerator.outputName}, {[basisName 'DD']}, Data.DataFunctionType.PER_EPISODE );
            
            % Post call to get the derivatives in time
            % \frac{ \par Basis }{ \par phase} * frac{ \par phase }{ dt }
            obj.addDataManipulationFunction('basisPhaseDtoTimeD', {[basisName 'D'],[phaseGenerator.outputName 'D']}, {[basisName 'D']}, Data.DataFunctionType.PER_EPISODE );
            obj.addDataManipulationFunction('basisPhaseDDtoTimeDD', ...
                {[basisName 'D'], [basisName 'DD'], [phaseGenerator.outputName 'D'], [phaseGenerator.outputName 'DD']}, {[basisName 'DD']}, Data.DataFunctionType.PER_EPISODE );
            obj.addDataFunctionAlias( 'generateBasisD', 'basisPhaseDtoTimeD');
            obj.addDataFunctionAlias( 'generateBasisDD', 'basisPhaseDDtoTimeDD');
        end
        
        function basisD = basisPhaseDtoTimeD (obj, basisD_ph, phaseD)
            basisD =  bsxfun(@times, basisD_ph, phaseD);
        end
        
        function basisDD = basisPhaseDDtoTimeDD (obj, basisD, basisDD_ph, phaseD, phaseDD)
            basisD_ph = bsxfun(@rdivide, basisD, phaseD);
            basisDD   = bsxfun(@times, basisDD_ph, phaseD.^2) + bsxfun(@times, basisD_ph, phaseDD );
        end
        
    end
    
    methods (Abstract)
        basis   = generateBasis(obj, phase)
        basisD  = generateBasisD(obj, phase)
        basisDD = generateBasisDD(obj, phase)
    end
end
