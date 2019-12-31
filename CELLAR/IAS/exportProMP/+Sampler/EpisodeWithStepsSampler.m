classdef EpisodeWithStepsSampler < Sampler.EpisodeSampler
% The EpisodeWithStepsSampler is able to run a number of sequential Episodes.
%
% This Sampler is a Episode Sampler that is equipped with a Sampler.StepSampler. 
% Allowing it to manage a number of sequential episodes as a Episode Sampler 
% and let the StepSampler handle the Episodes itself.
%
% The Stepsampler will be executed in the ‘Episodes’ Pool defined in the 
% masterclass Sampler.EpisodeSampler with priority 5.
    properties (SetAccess = protected, GetAccess = public)
         stepSampler
    end
    
    properties

    end
    
    methods
        function [obj] = EpisodeWithStepsSampler(dataManager, samplerNameEpisodes, samplerNameSteps)
            % @param dataManager Data.DataManager this sampler operates on
            % @param samplerName name of this sampler
            % @param samplerNameSteps name of the included Sampler.StepSampler
            if (~exist('samplerNameEpisodes', 'var'))
                samplerNameEpisodes = 'episodes';
            end
            
            if (~exist('samplerNameSteps', 'var'))
                samplerNameSteps = 'steps';
            end
            
            if (~exist('dataManager', 'var'))
                dataManager = [];
            end
            
            obj = obj@Sampler.EpisodeSampler(dataManager, samplerNameEpisodes);
            obj.createStepSampler(dataManager, samplerNameSteps);
            obj.dataManager.setSubDataManager(obj.stepSampler.getDataManagerForSampler());
            obj.addSamplerFunctionToPool('Episodes', samplerNameSteps, obj.stepSampler);                    
        end
        
        function [] = createStepSampler(obj, varargin)
            obj.stepSampler = Sampler.StepSampler(varargin{:});            
        end
        
        function [] = copyPoolsFromSampler(obj, sampler)
            obj.copyPoolsFromSampler@Sampler.EpisodeSampler(sampler);
            obj.flushSamplerPool('Episodes');
            obj.addSamplerFunctionToPool('Episodes', obj.stepSampler.samplerName, obj.stepSampler);
            obj.stepSampler.copyPoolsFromSampler(sampler.stepSampler);
        end
        
        function [dataManager] = getStepDataManager(obj)
            dataManager = obj.stepSampler.getDataManagerForSampler();
        end
        
        function [stepSampler] = getStepSampler(obj)
            stepSampler = obj.stepSampler;
        end
        
        function setStepSampler(obj, stepSampler)
            obj.stepSampler = stepSampler;
        end
        
                                        
        function [] = setActionPolicy(obj, actionPolicy)
            obj.stepSampler.setPolicy(actionPolicy);
        end
        
        function [] = setInitialStateSampler(obj, initStateSampler, varargin)
            obj.stepSampler.setInitialStateSampler(initStateSampler, varargin{:});
        end
        
        function [] = setTransitionFunction(obj, transitionFunction)
            obj.stepSampler.setTransitionFunction(transitionFunction);
        end               
        
        function [] = setRewardFunction(obj, rewardFunction)
            obj.stepSampler.setRewardFunction(rewardFunction);
            
            if (rewardFunction.isSamplerFunction('sampleFinalReward'))
                obj.setFinalRewardFunction(rewardFunction);
            end
        end
        
        function [numSamples] = getNumSamples(obj, data, varargin)
            numSamples = obj.getNumSamples@Sampler.EpisodeSampler(data, varargin{:});
            
            numSamples(2) = obj.stepSampler.isActiveSampler.toReserve();
        end
        
    end
end