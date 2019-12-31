classdef ActivationGenerator < Data.DataManipulator

    
    properties
        numComp;
        type; % 1 ALL_ACTIV
              % 2 BLEND2
              % 3 CUSTOM
        customActiv;
    end
    
    methods
        
        function obj = ActivationGenerator( dataManager, numComp, type, customActiv )
            
            dataManager.addDataEntry( 'steps.actvFactor',  numComp );        
            dataManager.addDataEntry( 'steps.actvFactorD', numComp );
            obj.addDataManipulationFunction('generateActivation',  {'phase', 'phaseD'}, ...
                                        {'actvFactor', 'actvFactorD'},  Data.DataFunctionType.PER_EPISODE );
            
            
            if ~exist('customActiv','var')          
                obj.numComp = numComp;
                assert( strcmp(type,'ALL_ACTIV') || strcmp(type,'BLEND2') );
                if strcmp(type,'ALL_ACTIV')
                    obj.type = 1; % ALL_ACTIV
                else
                    obj.type = 2; % BLEND2
                end
            else
                obj.type = 3; % CUSTOM
                obj.customActiv = customActiv;
            end
            
        end
        
        function [actvFactor, actvFactorD] = generateActivation(obj, phase, phaseD) 
            
            if ( obj.type == 3 ) % CUSTOM           
                actvFactor = obj.customActiv;  
                
            elseif ( obj.type == 2 )  % BLEND2              
                assert(obj.numComp == 2);   
                
                x = exp(-(phase - 0.5)* 8*40);
                actvFactor = 1 ./ (1 + x);
                actvFactorD = 8*40 * phaseD .* x ./ ( 1 + x ).^2;
                
                actvFactor  = [actvFactor,  1 - actvFactor ];
                actvFactorD = [actvFactorD,   - actvFactorD];
                
            else                
                actvFactor = ones(length(phase), obj.numComp);% ./ obj.numComp;
                actvFactorD = zeros(size(actvFactor));
            end            
        end
        
    end
    
end
