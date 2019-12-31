classdef SequentialSampler < Sampler.Sampler
    % SequentialSampler serves as the base class for every sequential sampler. 
    %
    % To create samples this class will go through the following steps :
    % - reserving storage specified by obj.IsActiveSampler (Default:Sampler.isActiveSetpSampler)
    % - calling the abstract function initSamples
    % - call the abstract functions createSamplesForStep and endTransition
    %   for each step for every episode checking via isActiveSampler witch 
    %   of those episodes still need processing
    properties (SetAccess = protected, GetAccess = public)
        isActiveSampler;
    end
    
    properties (Access = protected)
        transitionElementOldStep = {};
        transitionElementNewStep = {};
    end
    
    
    properties(SetObservable,AbortSet)
        
    end
    
    methods
        
        function [obj] = SequentialSampler(dataManager, samplerName, varargin)
            % @param dataManager Data.DataManager this sampler operates on
            % @param samplerName name of this sampler
            % @param varargin name of the steps (default: 'timeSteps')
            obj = obj@Sampler.Sampler(dataManager, samplerName);
            obj.setIsActiveSampler(Sampler.IsActiveStepSampler.IsActiveNumSteps(dataManager, varargin{:}));
        end
        
        function [] = addElementsForTransition(obj, transitionElementOld, transitionElementNew)
            obj.transitionElementOldStep{end + 1} = transitionElementOld;
            obj.transitionElementNewStep{end + 1} = transitionElementNew;
        end
        
        function [] = createSamples(obj, data, varargin)
            % @param data the Data.Data structure the sampler operates on
            % @param varargin hierarchical indicing of the data structure
            %
            % The SequentialSampler creates samples by firstly initiate the
            % data and after that run the appropriate sampler pools for each
            % step and, after checking witch episodes are still active, copy 
            % the transition elements for the new step to the next step.
            %
            % It does that by Initialising the dataManger via <tt>initSamples()</tt>
            % and running createSamplesForStep and endTransition for each
            % step sequentially while obj.isActiveSampler determines which 
            % of the Episodes are still active and need processing. If 
            % obj.isActiveSampler returns an empty vector, meaning every
            % episode has terminated, the sampling is done.
            layerIndex = varargin;
            
            reservedStorage = obj.isActiveSampler.toReserve();
            data.reserveStorage(reservedStorage, layerIndex{:});
            %             data.resetFeatureTags();
            
            activeIndex = layerIndex;
            activeIndex{length(varargin) + 1} = 1;
            
            %[activeIndex] = selectActiveIdxs(obj, data, activeIndex{:});
            obj.initSamples(data, activeIndex{:});
            
            step = 1;
            
            finished = false;
            numSteps = obj.getNumSamples(data, varargin{:});
            while(~finished)
                
                activeIndex{end} = step ;
                obj.createSamplesForStep(data, activeIndex{:});
                 
                step = step + 1;
                activeIndexNew = obj.selectActiveIdxs(data, activeIndex{:});
                
                if( step > reservedStorage && all(cellfun(@numel, activeIndexNew) > 0))
                    reservedStorage = reservedStorage*2;
                    numSteps(1) = reservedStorage;
                    data.reserveStorageNoReserveOld(numSteps, activeIndexNew{1:end-1});
                end
                
                activeIndex = activeIndexNew;
                finished = any(cellfun(@numel, activeIndex) == 0);
                if(~finished )
                    obj.endTransition(data, activeIndex{:});
                end
            end
            
        end
        
        
        
        function [numSamples] = getNumSamples(obj, data, varargin)
            numSamples = obj.isActiveSampler.toReserve;
        end
        
        function [activeIdxs] = selectActiveIdxs(obj, data, varargin)
            % assumes varargin{end-1} is a vector
            % all others are scalars
            isActive = obj.isActiveSampler.callDataFunctionOutput('isActiveStep', data, varargin{:});
            
            tcurrent = varargin{end};
            
            if (length(varargin) > 2)
                if (length(varargin{1}) > 1)
                    activeIdxs = {varargin{1:end-2}(isActive), varargin{end-1}, tcurrent};
                    stoppedIdxs = {varargin{1:end-2}(~isActive), varargin{end-1}, tcurrent};            
                else
                    activeIdxs = {varargin{1:end-2}, varargin{end-1}(isActive), tcurrent};
                    stoppedIdxs = {varargin{1:end-2}, varargin{end-1}(~isActive), tcurrent};            
                end
            else
                activeIdxs = {varargin{end-1}(isActive), tcurrent};
                stoppedIdxs = {varargin{end-1}(~isActive), tcurrent};            
            end
            if(any(~isActive))
                data.reserveStorage(tcurrent, stoppedIdxs{1:end-1});
            end
        end
        
        %function [] = setGetActiveFunction(obj, isActiveFunction)
        %    obj.flushSamplerPool('RewardSampler');
        %    obj.addSamplerFunction('RewardSampler', 'isActiveStep', isActiveFunction);
        %end
        function [] = setIsActiveSampler(obj, sampler)
            obj.isActiveSampler = sampler;
        end
        
    end
    
    methods (Access=protected)
        function [] = endTransition(obj, data, varargin)
            layerIndex = varargin;
            layerIndexNew = layerIndex;
            layerIndexNew{end} = layerIndexNew{end} + 1;
            
            for i = 1:length(obj.transitionElementOldStep)
                elementNextTimeStep = data.getDataEntry(obj.transitionElementOldStep{i}, layerIndex{:});
                numElements = size(elementNextTimeStep,1);
                data.setDataEntry(obj.transitionElementNewStep{i}, elementNextTimeStep, layerIndexNew{:});
            end
        end
    end
    
    methods (Abstract, Access=protected)
        
        
        
        [] = initSamples(obj, data, varargin)
        [] = createSamplesForStep(obj, data, varargin)
        
    end
    
end