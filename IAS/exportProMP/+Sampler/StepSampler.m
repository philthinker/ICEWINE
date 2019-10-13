classdef StepSampler < Sampler.SequentialSampler
    % The StepSampler is a Sampler.SequencialSampler, hence some of his
    % sampling pools will be called in sequential order.
    %
    %
    % After the Sampler has been set up correctly, by adding the suitable
    % functions into the sampler pools this sampler will do the following
    % steps to create the samples: 
    %
    % First call <tt>initSamples()</tt> which will execute the sampling
    % pool ‘InitSamples’ to create an initial state for the stepsampler.
    % Secondly, for each step, begin with calling <tt>createSamplesForStep()</tt>
    % to execute the remaining pools (more explanation follows) and then 
    % after checking if any episode is terminaed via isActiveSampler, transfer
    % the data from <tt>‘nextStates’</tt> in the current step to <tt>‘states’</tt> in the next step.
    %
    % The sampler pools in the second step, if properly configured, should
    % to the following: 
    %
    %
    %  - <tt>Policy</tt> (Priority: 20): This sampler pool determines the
    % actions of the agent within the learning process.
    %
    %  - <tt>TransitionSampler</tt>(Priority: 50): This  pool simulates
    % the environment, by computing the <tt>nextStates</tt> dependent on
    % the <tt>states</tt> and the actions given by the policy sampler pool.
    % 
    %  - <tt>RewardSampler</tt>(Priority: 80): This pool evaluates the reward
    % for the sampler in each step.
    %
    % In addition to the data manipulation functions above this sampler 
    % also adds the data entries <tt>’states’</tt>, <tt>’nextStates’</tt>
    % and <tt>’timeSteps’</tt>.
    properties (Access = protected)
       
    end
        
    methods
        function [obj] = StepSampler(dataManager, samplerName)
            % @param dataManager Data.DataManager this sampler operates on
            % @param samplerName name of this sampler
            if (~exist('samplerName', 'var'))
                samplerName = 'steps';
            end
            
            if (~exist('dataManager', 'var') || isempty(dataManager))
                dataManager = Data.DataManager(samplerName);
            else
                dataManager = dataManager.getDataManagerForName(samplerName);
            end
                        
            obj = obj@Sampler.SequentialSampler(dataManager, samplerName);
            obj.addSamplerPool('InitSamples', 1);
            obj.addSamplerPool('Policy', 20);
            obj.addSamplerPool('TransitionSampler', 50);
            obj.addSamplerPool('RewardSampler', 80);
            
            obj.addElementsForTransition('nextStates', 'states');        
            obj.dataManager.addDataEntry('timeSteps', 1);                                  
        end               
        
        %%  Sampler Pools add, flush, set ( flush and set )
        function [] = setTransitionFunction(obj, transitionFunction, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleNextState';
            end
            obj.addSamplerFunctionToPool( 'TransitionSampler', samplerName, transitionFunction, 0);            
        end
        
        function [] = setPolicy(obj, policy, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleAction';
            end
            obj.addSamplerFunctionToPool( 'Policy', samplerName, policy, 0);            
        end
        
        function [] = setRewardFunction(obj, rewardFunction, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleReward';
            end
            obj.addSamplerFunctionToPool( 'RewardSampler', samplerName, rewardFunction, 0);            
        end
        
        function [] = setInitialStateSampler(obj, initStateSampler, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleInitState';
            end
            obj.addSamplerFunctionToPool( 'InitSamples', samplerName, initStateSampler, 0);            
        end                
        
        function [] = flushTransitionFunction(obj)
            obj.flushSamplerPool('TransitionSampler');
        end
        
        function [] = flushPolicy(obj)
            obj.flushSamplerPool('Policy');      
        end
        
        function [] = flushRewardFunction(obj)
            obj.flushSamplerPool('RewardSampler');        
        end
        
        function [] = flushInitialStateSampler(obj)
            obj.flushSamplerPool('InitSamples');      
        end   
        
        %%
    end
    
    methods (Access = protected)
        function [] = endTransition(obj, data, varargin)
            obj.endTransition@Sampler.SequentialSampler(data, varargin{:});
            
            layerIndexNew = varargin;
            layerIndexNew{end} = layerIndexNew{end} + 1;            
            numElements = data.getNumElementsForIndex(length(varargin), varargin{:});
            data.setDataEntry('timeSteps', ones(numElements,1) * layerIndexNew{end}, layerIndexNew{:});
        end
        
        function [] = initSamples(obj, data, varargin)
            %initStates = data.getDataEntry('initStates', varargin{1:end-1});
            %data.setDataEntry('states', initStates, varargin{:});
            %data.setDataEntry('timeSteps', 1, varargin{:});
            obj.createSamplesFromPool('InitSamples', data, varargin{:});
            numElements = data.getNumElementsForIndex(length(varargin),varargin{:});
            data.setDataEntry('timeSteps',  ones(numElements,1) , varargin{:});
        end
        
        function [] = createSamplesForStep(obj, data, varargin)
            %sample policy, transition, reward
            obj.createSamplesFromPoolsWithPriority(10, 90, data, varargin{:});
        end
        

         
    end
    
end