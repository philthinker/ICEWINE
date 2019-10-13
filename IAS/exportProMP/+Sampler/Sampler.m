classdef Sampler < Sampler.SamplerInterface
    % Sampler.Sampler serves as the base class for all other Samplers.
    %
    % If you want to inherent form this class to make a new sampler class 
    % you should consider using the subclasses Sampler.SequentialSampler
    % and Sampler.IndependentSampler, or even consider Sampler.EpisodeSampler and 
    % Sampler.EpisodeWithStepSampler if you want to use the learning scenario.
    %
    % Every Sampler models a sampling scenario. Various sampler pools will
    % be used to handle action policy, simulating the environment, rewards
    % and such. Each of those pools may contain a number of sampler functions 
    % acting on a given Data.Data structure. The task of the Sampler is 
    % organizing those pools and their execution.
    %
    % Since it is a subclass of <tt>Data.DataManipulator</tt> it uses
    % DataManipulationFunctions to act on its given dataManager. Those are
    % combined into samplerpools. Each of those pools is given a certain priority.
    %
    % Important functions in this class are:
    %
    % -addSamplerPool: adds a new empty sampler pool to the sampler.
    %
    % -addSamplerFunction: adds a sampler function to an existing sampler pool.
    %
    % -sampleAllPools: executes all pools in the sampler in order of priority.
    % Beginning with the lowest priority .
    %
    % -createSamplesFromPoolsWithPriority: executes all pools in a specific priority range.
    %
    % -createSamplesFromPool: executes a pool in the sampler.

    properties (SetAccess = protected)
        samplerPools % Container Map of all  functions in a given pool
        
        samplerName % string of the sampler name
        samplerPoolPriorityList % list of all pools ordered by priority 
        
        lowerLevelSamplers; % list of added lower level Samplers 
        
        samplerMap; % list of lower level Samplers after finalizing the sampler
    end
    
    methods
        function [obj] = Sampler(dataManager, samplerName)
            % @param dataManager Data.DataManager this sampler operates on
            % @param samplerName name of this sampler
            obj = obj@Sampler.SamplerInterface(dataManager);
            
            obj.samplerPools = containers.Map();
            obj.samplerPoolPriorityList = {};
            
            obj.samplerName = samplerName;
            obj.samplerMap = containers.Map;
        end
        
        function [] = setSamplerIteration(obj, iteration)
            obj.iterIdx = iteration;
        end
        
        function [dataManager] = getDataManagerForSampler(obj)
            dataManager = obj.dataManager;
        end
        
        function [] = finalizeSampler(obj, finalizeData)            
            lowLevelSamplers = obj.getLowLevelSamplers();
            for i = 1:length(lowLevelSamplers)
                if (obj.samplerMap.isKey(lowLevelSamplers(i).getSamplerName()))
                    error('Sampler %s exists twice as low-level sampler!', lowLevelSamplers(i).getSamplerName());
                end
                lowLevelSamplers(i).finalizeSampler(false);
                obj.samplerMap(lowLevelSamplers(i).getSamplerName()) = lowLevelSamplers(i);
            end
            if (~exist('finalizeData', 'var'))
                finalizeData = true;
            end
            if (finalizeData)
                obj.dataManager.finalizeDataManager();
            end
        end
        
        function [] = copyPoolsFromSampler(obj, sampler)
            if (~sampler.samplerPools.isempty())
                obj.samplerPools = containers.Map(sampler.samplerPools.keys, sampler.samplerPools.values);
            else
                obj.samplerPools = containers.Map();
            end
            obj.samplerPoolPriorityList = sampler.samplerPoolPriorityList;
            %sampler.lowerLevelSamplers = obj.lowerLevelSamplers;
            if (~sampler.samplerMap.isempty())
                obj.samplerMap = containers.Map(sampler.samplerMap.keys, sampler.samplerMap.values);
            else
                obj.samplerMap = containers.Map();
            end
        end

        function [] = copySamplerFunctionsFromPool(obj, sampler, poolName)
            % @param sampler the sampler to copy the pools from
            % @param poolName the name of the new pool in this sampler
            %
            % Copies all sampler functions from 'poolName' in sampler
            % to same pool in 'this' sampler
            obj.samplerPools(poolName) = sampler.samplerPools(poolName);
        end
            
        function [isSampler] = isSamplerFunction(obj, samplerName)
            if (strcmp(samplerName, obj.samplerName) == true)
                isSampler = true;
            else                
                isSampler = obj.isSamplerFunction@Data.DataManipulator(samplerName);
            end                
        end
        
        function [] = callDataFunction(obj, name, newData, varargin)
            if (strcmp(name, obj.samplerName) == true)
                obj.createSamples(newData, varargin{:});
            else
                obj.callDataFunction@Data.DataManipulator(name, newData, varargin{:});
            end
        end
        
        function [] = addSamplerPool(obj, samplerPoolName, priority) 
            % @param samplerPoolName name of the new pool
            % @param priority priority of the new parameter
            %
            % Creating a new empty sampler pool. The new sampler pool will
            % be inserted into the samplerPoolPriorityList according to
            % given priority.
            %           
            % An error will occur if a sampler pool with the same name
            % or the same priority already exist.
            
            if (obj.samplerPools.isKey(samplerPoolName))
                error('Sampler pool with the name %s already exist!', samplerPoolName);
            end
            
            % TODO CHECK FOR POOLS WITH SAMPLE PRIORITY

            samplerPool.samplerList = {};
            samplerPool.poolName = samplerPool;
            samplerPool.priority = priority;
            
            obj.samplerPools(samplerPoolName) = samplerPool;
            
            index = 0;
            priorityTmp = -inf;
            while (priority > priorityTmp && index <= length(obj.samplerPoolPriorityList))
                index = index + 1;
                if (index <= length(obj.samplerPoolPriorityList))
                    priorityTmp = obj.samplerPools(obj.samplerPoolPriorityList{index}).priority;                
                end
            end
            if (index == 0)
                index = 1;
            end
            obj.samplerPoolPriorityList = {obj.samplerPoolPriorityList{1:index-1}, samplerPoolName, obj.samplerPoolPriorityList{index:end}};
                
        end
        
        function [] = flushSamplerPool(obj, samplerPoolName)  
            % @param samplerPoolName name of the pool to flush
            %
            % remove all functions in given pool
            samplerPool = obj.samplerPools(samplerPoolName);
            samplerPool.samplerList = {};
            obj.samplerPools(samplerPoolName) = samplerPool;
        end
        
        function [] = addLowerLevelSampler(obj, samplerPool, lowerLevelSampler, isBeginning)
            obj.addSamplerFunction(samplerPool, lowerLevelSampler.getSamplerName(), lowerLevelSampler, isBeginning);
            if (isempty(obj.lowerLevelSamplers))
                obj.lowerLevelSamplers = lowerLevelSampler;
            else
                obj.lowerLevelSamplers(end + 1) = lowerLevelSampler;
            end
        end
        
        function [lowLevelSamplers] = getLowLevelSamplers(obj)
            lowLevelSamplers = obj.lowerLevelSamplers;
            for i = 1:length(obj.lowerLevelSamplers)
                lowLevelSamplers = [lowLevelSamplers, obj.lowLevelSamplers(i).getLowLevelSamplers()];
            end
        end        
        
        function [] = addSamplerFunctionToPool(obj, samplerPool, samplerName, objHandle, addLocationFlag)
            % @param samplerPool sampler pool to whom the sampler function will be included
            % @param samplerName name of the new sampler function
            % @param objHandle function handle of the new sampler function
            % @param addLocationFlag The value of addLocationFlag determines the behavior of the function: -1 will add the new samplerfunction to the beginning of the sampler pool, 0 will flush the entire sampler pool and 1 will add the new sampler function at the end of the sampler pool.
            %
            % add a function or a sampler to a sampler pool.
            %                    
            % If the paramter addLocationFlag is not given the new sampler function will be added to the end of the sampler pool, meaning it will be executed last. If addLocationFlag is given the the follwing will happen:
            %
            % - addLocationFlag = -1: add the new sampler function in beginning of the pool
            %
            % - addLocationFlag = 0: flush the entire sampler pool
            %
            % - addLocationFlag = 1: add the new sampler function at end of the sampler pool(default)

            
            if (~objHandle.isSamplerFunction(samplerName))
                error('%s is not a valid sampler function of the object\n', samplerName);
            end
            
            if (~exist('addLocationFlag', 'var'))
                addLocationFlag = 1;
            end
            
            sampleFunction.samplerName = samplerName;
            sampleFunction.objHandle = objHandle;
            
            pool = obj.samplerPools(samplerPool);
            switch (addLocationFlag)
                
                case -1
                    pool.samplerList =  [{sampleFunction}, pool.samplerList{:}];                    
                case 0
                    pool.samplerList =  {sampleFunction};
                case 1           
                    pool.samplerList{end + 1} = sampleFunction;
            end
            
            obj.samplerPools(samplerPool) = pool;
        end
        
        function [] = createSamplesFromPool(obj, poolName, data, varargin)
            % @param poolName name of the selected pool
            % @param data the Data.Data structure the pool operates on
            % @param varargin hierarchical indicing of the data structure
            % 
            % executes all functions on the samplerlist of a given pool
            samplerPool = obj.samplerPools(poolName);
            for i = 1:length(samplerPool.samplerList)
                samplerName = samplerPool.samplerList{i}.samplerName;
                objectPointer = samplerPool.samplerList{i}.objHandle;
                objectPointer.callDataFunction(samplerName, data, varargin{:});
            end
        end
        
        function [] = sampleAllPools(obj, newData, varargin) 
            % @param newData the Data.Data structure the pool operates on
            % @param varargin hierarchical indicing of the data structure
            %
            % executes all functions in every pool in this sampler
            for i = 1:length(obj.samplerPoolPriorityList)
                obj.createSamplesFromPool(obj.samplerPoolPriorityList{i}, newData, varargin{:});
            end                    
        end
        
        function [] = createSamplesFromPoolsWithPriority(obj, lowPriority, highPriority, newData, varargin)
            % @param lowPriority lower bound of the priority of the pools that will be executed 
            % @param highPriority upper bound of the priority of the pools that will be executed 
            % @param newData the Data.Data structure the pool operates on
            % @param varargin hierarchical indicing of the data structure
            %
            % executes all functions on the samplerlist of a given pool in
            % a specific priority Range
            for i = 1:length(obj.samplerPoolPriorityList)
                priorityTmp = obj.samplerPools(obj.samplerPoolPriorityList{i}).priority;  
                if (priorityTmp >= lowPriority && priorityTmp <= highPriority)
                    obj.createSamplesFromPool(obj.samplerPoolPriorityList{i}, newData, varargin{:});
                end
            end
        end
                                 
    end
    
    methods (Access = protected)
        
        function [] = addSamplerToPoolInternal(obj, poolName, samplerName, samplerObj, flush)
            
            if ( exist('flush','var') && flush ~= 0 )
                obj.flushSamplerPool( poolName );
            end          
            
            obj.addSamplerFunction( poolName, samplerName, samplerObj);
            
        end        
        
    end
    
    methods (Abstract)
        numSamples = getNumSamples(obj, data, index)
    end
end