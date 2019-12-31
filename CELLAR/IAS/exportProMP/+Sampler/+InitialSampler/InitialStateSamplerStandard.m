classdef InitialStateSamplerStandard < Sampler.InitialSampler.InitialStateSampler
    % The InitialStateSamplerStandard class provides random distributed
    % values for sampling.
    %
    % The parameters of this sampler have to be changed by a Settings
    % Object. The following properties can be set: 
    %
    % InitialStateDistributionMinRange(Default: -1): Set the minimum Range
    % for the random values, if the statespace is multidimensional it may
    % be a vector setting the range for each dimension.
    %
    % InitialStateDistributionMaxRange(Default: 1): Set the maximum Range
    % for the random values, if the statespace is multidimensional it may
    % be a vector setting the range for each dimension.
    %
    % InitialStateDistributionType(Default: Gaussian): Set the random
    % number generator to an uniform or a gaussian distribution. The
    % setting is ‘Uniform’ or ‘Gaussian’ respectively.
    
    properties(SetObservable,AbortSet)
        InitialStateDistributionMinRange = -1; % lower boundary of the RNG
        InitialStateDistributionMaxRange = 1;  % upper boundary of the RNG
        
        InitialStateDistributionType = 'Gaussian'; % Type of RNG can be set to ‘Uniform’ or ‘Gaussian’
    end
    
    methods
        function [obj] = InitialStateSamplerStandard(dataSampler)
            
                        
            obj = obj@Sampler.InitialSampler.InitialStateSampler(dataSampler);
            obj.linkProperty('InitialStateDistributionMinRange');
            obj.linkProperty('InitialStateDistributionMaxRange');
            
            obj.linkProperty('InitialStateDistributionType');    
                        
            % TODO: check where Chris needs that
            %level = obj.dataManager.getDataManagerDepth('steps') - 1;
             
            %obj.registerOptionalParameter('StartPos', false, numJoints, -3.14 * ones(1, numJoints), 3.14 * ones(1, numJoints), 'contexts', level);
            %obj.registerOptionalParameter('StartVel', false, numJoints, -10 * ones(1, numJoints), 10 * ones(1, numJoints), 'contexts', level);
 
            %obj.setIfNotEmpty('StartPos',  zeros(1, numJoints));
            %obj.setIfNotEmpty('StartVel',  zeros(1, numJoints));                                   

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
            numDimTaken = 0;
            dimState = obj.dataManager.getNumDimensions('states');
            states = zeros(numElements, dimState);
            if (~isempty(varargin))
                context = varargin{1};
                numDimTaken = min(size(states, 2), size(context,2));
                
                states(:, 1:numDimTaken) = context(:, 1:numDimTaken);
            end
            
            minRange = obj.InitialStateDistributionMinRange;
            maxRange = obj.InitialStateDistributionMaxRange;
            
            if (numel(minRange) == 1)
                minRange = ones(1, dimState) * minRange;
                maxRange = ones(1, dimState) * maxRange;
            end
            
            minRange = minRange((numDimTaken + 1):end);
            maxRange = maxRange((numDimTaken + 1):end);
            switch (obj.InitialStateDistributionType)
                case 'Gaussian'
                    randFunc    = @randn;
                    width       = (maxRange - minRange) / 2;
                    offset      =  (maxRange + minRange) / 2;
                    
                case 'Uniform'
                    randFunc    = @rand;
                    width       = maxRange - minRange;
                    offset      = minRange;
            end
          
            states(:, (numDimTaken + 1):end) = bsxfun(@plus, bsxfun(@times, randFunc(numElements, dimState - numDimTaken), width), offset);

        end
    end
end