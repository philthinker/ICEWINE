classdef ProMPsModelFree < TrajectoryGenerators.ProMPs 
    
    properties        
        numObs;
    end
    
    methods
        
        function obj = ProMPsModelFree(dataManager, numObs, numCtl, varargin )
            
            obj = obj@TrajectoryGenerators.ProMPs(dataManager,numCtl+numObs, varargin{:});
            
            obj.numObs = numObs;
            
        end
        
        %% Distributions.Distribution implementations
        
        function [refState] = sampleFromDistribution(obj, varargin)
        
            refState = sampleFromDistribution@TrajectoryGenerators.ProMPs(obj,varargin{:});
        
            refState = refState(:,1:(2*obj.numObs) );
        
        end
        
        
    end
    
end
