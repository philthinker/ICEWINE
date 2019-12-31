classdef DistributionWithMeanAndVariance < Distributions.Distribution
    % The DistributionWithMeanAndVariance is a subclass of Distribution and
    % augments the superclass with Mean and Variance.
    %
    % This class defines <tt>sampleFromDistribution()</tt> and <tt>getDataProbabilities()</tt>,
    % since this is a subclass of Distribution. The output of these functions
    % depends on the abstract function <tt>getExpectationAndSigma()</tt>,
    % which will determine the type of distribution in further subclasses.
    %
    % The abstract function <tt>getExpectationAndSigma(obj, numElements, varargin)</tt> 
    % is expected to return the following types of data: 
    %
    % - mean: Should be a matrix of size equal to numElements x dimension
    % denoting the expectation of the distribution given the inputData. 
    % - sigma: should be a 3 dimensional array, where the first dimension 
    % indicates the numElements and the last 2 dimensions contain 
    % the sigma matrices for the corresponding samples. There are 3 different types of 
    % sigma this class can handle: 
    %  + 1. Case: If the third dimension of the sigma matrix is 1, the 
    % class expects those values to be the diagonal variance.
    %  + 2. Case: If the first dimension of sigma is 1, there is only one
    % sigma matrix for all elements. Then the class uses this sigma matrix 
    % for every sample.
    %  + 3. Case (default): Every Element has its own sigma matrix and will
    % be handled as such.
    properties (SetAccess = protected)
        
    end
    
    properties
        restrictToRangeLogLik = true;
    end
    
    methods
        function obj = DistributionWithMeanAndVariance()
            obj = obj@Distributions.Distribution();
        end
        
        function [samples] = sampleFromDistribution(obj, numElements, varargin)
            % @param varargin parameter for the abstract <tt>getExpectationAndSigma()</tt>function. The first parameter is always <tt>numElements</tt>, the rest is dependent on the subclass you are using
            % returns a number of random samples of the distribution 
            % determined by the first parameter
            [expectation, sigma] = obj.getExpectationAndSigma(numElements, varargin{:});
            
            if (size(sigma, 3) == 1)
                % If the second dimension of the sigma matrix is 1, the
                % function expects those values to be the diagonal variance.
                samples = expectation + randn(size(expectation,1), size(expectation,2)) .* sigma;
            else
                                
                if (size(sigma,1) == 1)
                    % If the first dimension of sigma is 1, there is only
                    % one sigma matrix for all elements. Then we will 
                    % use this sigma matrix for every sample.
                    sigma = permute(sigma, [2 3 1]);
                    samples = expectation + randn(size(expectation)) * sigma;
                else
                    % Every Element has its own sigma matrix
                    samples = expectation;
                    for i = 1:size(samples,1)
                        samples(i,:) = samples(i,:) + randn(1, size(expectation,2)) * permute(sigma(i,:,:), [2, 3, 1]);
                    end
                end
            end
        end
        
        function [qData] = getDataProbabilities(obj, inputData, outputData, varargin)
            % @param inputData vector of input data
            % @param outputData vector of output data
            % @param varargin used in <tt>getExpectationAndSigma(numElements,inputData,varargin)</tt> 
            % returns a vector of the probability of inputData resulting in outputData.
            [expectation, sigma] = obj.getExpectationAndSigma(size(outputData,1), inputData, varargin{:});

            if (obj.restrictToRangeLogLik)
                minRange = obj.getDataManager().getMinRange(obj.outputVariable);
                maxRange = obj.getDataManager().getMaxRange(obj.outputVariable);
                expectation = bsxfun(@max, bsxfun(@min, expectation, maxRange), minRange);
            end

            if (size(sigma, 3) == 1)
                % If the second dimension of the sigma matrix is 1, the
                % function expects those values to be the diagonal variance.
                
                samples = outputData - expectation;
                samples = samples ./ sigma;
                
                qData = - sum(log(sigma),2);
            else
                % If the first dimension of sigma is 1, there is only
                % one sigma matrix for all elements. Then we will 
                % use this sigma matrix for every sample.
                
                if (size(sigma,1) == 1)
                    samples = outputData - expectation;
                    sigma = permute(sigma, [2, 3, 1]);
                    samples = samples / sigma;
                    
                    % Here we do not need the 0.5 as it is the standard
                    % deviation
                    qData = - sum(log(eig(sigma)));
                else
                    % Every Element has its own sigma matrix
                    samples = outputData - expectation;
                    qData = zeros(size(samples,1));
                    for i = 1:size(samples,1)
                        sigma_tmp = permute(sigma(i,:,:), [2, 3, 1]);
                        samples(i,:) = samples(i,:) ./ sigma_tmp;
                        qData(i) = - sum(log(eig(sigma_tmp)));                         
                    end
                end
            end
            samplesDist = sum(samples.^2, 2);
            %samplesDist = samplesDist - min(samplesDist);
            qData = -0.5 * samplesDist + qData - size(expectation,2)/2 * log(2*pi);      %Misssing 2 pi?      
        end
    end
    
    methods (Access=protected)
        function [] = registerMappingInterfaceDistribution(obj)
            obj.registerMappingInterfaceDistribution@Distributions.Distribution();
            %             obj.addMappingFunction('getExpectationAndSigma', );
            if (~isempty(obj.outputVariable))
                obj.restrictToRangeLogLik = obj.getDataManager().isRestrictToRange(obj.outputVariable);
            else
                obj.restrictToRangeLogLik = false;
            end
               
            if (obj.registerDataFunctions)
                obj.addDataManipulationFunction('getExpectationAndSigma', [obj.inputVariables, obj.additionalInputVariables], ...
                                              {[obj.outputVariable, 'Mean'],[obj.outputVariable, 'Std']}, Data.DataFunctionType.ALL_AT_ONCE, true);
            end
            
        end
    end
    
    methods (Abstract)
        % Check how this function is expected to behave in the documentation of this class
        [mean, sigma] = getExpectationAndSigma(obj, numElements, inputData, varargin);
    end
end
