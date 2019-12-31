classdef IndependentSampler < Sampler.Sampler
    % The IndependentSampler serves as base class for every sampler that 
    % does not need to sample each step sequentially.
    %
    % If the flag parallelSampling is true (default) the sampler will simply
    % run all sampler pools to create samples.
    %
    % If the flag parallelSampling is false the sampler will run each 
    % episode consecutively. This can be used if you are using a physical agent. 
    %
    % This sampler registers the following properties that can be manipulated 
    % via the Settings class:
    %
    %  - <tt>’numSamples’ </tt>: the number of episodes this sampler will run
    %
    %  - <tt>’numInitialSamples’ </tt>:  the number of steps this sampler will 
    % run before starting the learning sequence
    %
    %  - This sampler also adds the data entry <tt>’'iterationNumber'</tt> which 
    % counts the number of iterations this sampler has taken 
    properties (Access = protected)
         
        parallelSampling;        
    end
    
    properties(SetObservable,AbortSet)
        numSamples = 10;
        numInitialSamples = -1;
    end
    
    methods
        function [obj] = IndependentSampler(dataManager, samplerName)
            % @param dataManager Data.DataManager this sampler operates on
            % @param samplerName name of this sampler
            obj = obj@Sampler.Sampler(dataManager, samplerName);
            
            obj.parallelSampling = true;
                                    
            samplerNameUpper = [upper(samplerName(1)), samplerName(2:end)];
            obj.dataManager.addDataEntry('iterationNumber', 1);
            obj.linkProperty('numSamples', ['numSamples', samplerNameUpper]);
            obj.linkProperty('numInitialSamples', ['numInitialSamples', samplerNameUpper]);            
                        
        end
                                              
        function [] = setParallelSampling(obj, parallelSampling)
            obj.parallelSampling = parallelSampling;
        end
                                     
        function [] = createSamples(obj, newData, varargin)
            numSamples = obj.getNumSamples(newData, varargin{:});
                        
            if (numSamples > 0)
                newData.reserveStorage(numSamples, varargin{:});
                newData.resetFeatureTags();
                newData.setDataEntry('iterationNumber', obj.iterIdx);
                newIndex = varargin;
                if (obj.parallelSampling)

                    newIndex{end + 1} = 1:numSamples(1);
                    obj.sampleAllPools(newData, newIndex{:});
                else
                    index = 1;
                    while (index <= numSamples(1))
                        newIndex{length(varargin) +1} = index;
                        obj.sampleAllPools(newData, newIndex{:});
                        if (obj.isValidEpisode())
                            index = index + 1;
                        end
                    end
                end                                    
            end
        end
        
        function [isValid] = isValidEpisode(obj)
            isValid = true;
        end
        

        function [numSamples] = getNumSamples(obj, data, varargin)
            if length(obj.numSamples) == 1
                if (obj.iterIdx == 1 && obj.numInitialSamples > 0)
                    numSamples = obj.numInitialSamples;
                else
                    numSamples = obj.numSamples;                
                end
            else
                if (obj.iterIdx == 1 && obj.numInitialSamples > 0)
                    numSamples = obj.numInitialSamples;
                else
                    numSamples = obj.numSamples(obj.iterIdx);                
                end
            end
        end
    end
end