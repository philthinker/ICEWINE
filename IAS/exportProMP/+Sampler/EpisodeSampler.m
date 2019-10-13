classdef EpisodeSampler < Sampler.IndependentSampler
    % EpisodeSampler is a subclass of IndependentSampler that allows for a
    % more convenient implementation of an independent sampling.
    %
    % There are a number of SamplingPools  and corresponding  access
    % functions predefined. The Pools and their priority are
    % defined as follows:
    %
    % - InitEpisode (Priority 1):  Sets the starting conditions for the
    % episode(s). For example: a random starting position state for each episode.
    %
    % - Policy (Priority 3):  Determine the actions of the agent, usually 
    % depended on the actual state.
    %
    % - Episodes (Priority 5):  Run the sampler that handles each episode
    %
    % - FinalReward (Priority 6): Evaluates the result of each episode and
    % returns an additional reward.
    %
    % - Return (Priority 7):  Calculates the return of each episode by 
    %  summing the reward and the final reward
    
    properties (Access = protected)
        
    end
    
    properties
        contextDistribution
        returnSampler
        parameterPolicy
    end
    
    methods
        function [obj] = EpisodeSampler(dataManager, samplerName)
            % @param dataManager Data.DataManager this sampler operates on
            % @param samplerName name of this sampler
            if (~exist('samplerName', 'var'))
                samplerName = 'episodes';
            end
            
            if (~exist('dataManager', 'var') || isempty(dataManager))
                dataManager = Data.DataManager(samplerName);
            end
                                    
            obj = obj@Sampler.IndependentSampler(dataManager, samplerName);
            obj.dataManager.addDataAlias('contexts', {});
            
            obj.addSamplerPool('InitEpisode', 1);
            obj.addSamplerPool('ParameterPolicy', 3);
            obj.addSamplerPool('Episodes', 5);
            obj.addSamplerPool('FinalReward', 6);
            obj.addSamplerPool('Return', 7);
            
        end
        
        function [dataManager] = getEpisodeDataManager(obj)
            dataManager = obj.dataManager;
        end
        
        %%  Sampler Pools add, flush, set ( flush and set )
        
        function [] = setReturnFunction(obj, rewardSampler, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleReturn';
            end
            if (strcmp(samplerName, 'sampleReturn'))
                obj.returnSampler = rewardSampler;
            end
            
            obj.addSamplerFunctionToPool('Return', samplerName, rewardSampler, -1);
        end
        
        function [] = setFinalRewardFunction(obj, rewardSampler, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleFinalReward';
            end
            obj.addSamplerFunctionToPool('FinalReward', samplerName, rewardSampler, -1);
        end
        
        function [] = setParameterPolicy(obj, parameterSampler, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleParameter';
            end
            if (strcmp(samplerName, 'sampleParameter'))
                obj.parameterPolicy = parameterSampler;
            end
            
            obj.addSamplerFunctionToPool( 'ParameterPolicy', samplerName, parameterSampler, -1);
        end
        
        function [] = setContextSampler(obj, contextSampler, samplerName)
            if ( ~exist('samplerName', 'var') || isempty(samplerName) )
                samplerName = 'sampleContext';
            end
            if (strcmp(samplerName, 'sampleContext'))
                obj.contextDistribution = contextSampler;
            end
            
            obj.addSamplerFunctionToPool( 'InitEpisode', samplerName, contextSampler,- 1);
        end
        
        
        
        function [] = flushReturnFunction(obj)
            obj.flushSamplerPool('Return');
        end
        
        function [] = flushFinalRewardFunction(obj)
            obj.flushSamplerPool('FinalReward');
        end
        
        function [] = flushParameterPolicy(obj)
            obj.flushSamplerPool('ParameterPolicy');
        end
        
        function [] = flushContextSampler(obj)
            obj.flushSamplerPool('InitEpisode');
        end
        
        %%
        
    end
end