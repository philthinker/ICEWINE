classdef InitialStateSamplerFromTrainingData < Sampler.InitialSampler.InitialStateSampler

    properties (SetObservable, AbortSet)
        useLastState = false;
    end
    
    properties
        trData;
    end
    
    methods
        function [obj] = InitialStateSamplerFromTrainingData(dataSampler,trData)
            
                        
            obj = obj@Sampler.InitialSampler.InitialStateSampler(dataSampler);
            
            obj.trData = trData;
            obj.linkProperty('useLastState');

        end
        
        function [] = registerInitStateFunction(obj)
            obj.setInputArguments('sampleInitState', {'contexts'});
        end
        
        function [] = setInitStateFromContext(obj, useContext)
            if (useContext)
                obj.setInputArguments('sampleInitState', {'contexts'});
            else
                obj.setInputArguments('sampleInitState', {});
            end
        end
        
        function [states] = sampleInitState(obj, numElements, varargin)
            
            init_tr = obj.trData.getDataEntry3D('states');
            
            if ( obj.useLastState )
                states = squeeze( init_tr(1:numElements,end,:));
            else
                states = squeeze( init_tr(1:numElements,1,:));
            end

        end
    end
end