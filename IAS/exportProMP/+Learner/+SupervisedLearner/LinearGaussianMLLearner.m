classdef LinearGaussianMLLearner < Learner.SupervisedLearner.LinearFeatureFunctionMLLearner
    % The <tt>LinearGaussianMLLearner</tt> is a supervised learner that uses
    % weighted maximum likelihood estimate of the covariance matrix.
    %
    % Just like its superclass <tt>LinearFeatureFunctionMLLearner</tt> this
    % class uses linear function approximator of the function class given via
    % constructor. In addition to its superclass we now need a gaussian 
    % distribution class , since this function needs to be equipped with 
    % <tt>setSigma()</tt> and <tt>setCovariance()</tt>.
    %
    % The <tt>learnFunction()</tt> will approximate the gaussian type 
    % distribution via weighted maximum likelihood estimate of the covariance matrix. The following 
    % settings have been registered and can be used to manipulate the learning
    % function. (see Common.Settings for more information on settings): 
    %
    % - <tt>maxCorr</tt> (default 1): maximum correlation coefficient in the covariance matrix.
    % - <tt>minCov</tt> (default 10^-6): minimal covariance of the diagonal elements in the sigma matrix
    % - <tt>priorCov</tt> (default 1): factor for the prior of the
    % - <tt>priorCovWeight</tt> (default 10^-16): influence of the prior
    
    properties (SetObservable,AbortSet)
        maxCorr
        minCov
        priorCov
        priorCovWeight
        KL = 1;
    end
    
    % Class methods
    methods
        function obj = LinearGaussianMLLearner(dataManager, linearfunctionApproximator, varargin)
            % @param dataManager Data.DataManger to operate on
            % @param linearfunctionApproximator function object that will  be learned needs to be of gaussian type
            % @param varargin contains the following optional arguments in this order: weightName, inputVariables, outputVariable (see superclass SupervisedLearner)
            obj = obj@Learner.SupervisedLearner.LinearFeatureFunctionMLLearner(dataManager, linearfunctionApproximator, varargin{:});
            
            mapName = linearfunctionApproximator.outputVariable;
            mapName(1) = upper(mapName(1));
            %meta = obj.addprop(['maxCorr', mapName]);
            %meta.SetObservable = true;
            %meta.AbortSet = true;
            obj.maxCorr = 1.0;            
            obj.linkProperty('maxCorr', ['maxCorr', mapName]);            
            
            %meta = obj.addprop(['minCov', mapName]);
            %meta.SetObservable = true;
            %meta.AbortSet = true;
            obj.minCov = 10^-12;

            obj.linkProperty('minCov', ['minCov', mapName]);            
            
            %meta = obj.addprop(['priorCov', mapName]);
            %meta.SetObservable = true;
            %meta.AbortSet = true;
            obj.priorCov = 1.0;
            obj.linkProperty('priorCov', ['priorCov', mapName]);            

            %meta = obj.addprop(['priorCovWeight', mapName]);
            %meta.SetObservable = true;
            %meta.AbortSet = true;
            obj.priorCovWeight = 10^-16;                                    
            obj.linkProperty('priorCovWeight', ['priorCovWeight', mapName]);            
        end
        
        function [] = learnFunction(obj, inputData, outputData, weighting)
            
            if(isempty(inputData))
                inputData = reshape(inputData, size(outputData, 1), 0);
            end
            
            if (~exist('weighting', 'var') || isempty(weighting) )
                weighting = ones(size(inputData,1),1);
            end
            oldMean = mean(obj.functionApproximator.getExpectation(size(inputData,1), inputData));
            obj.learnFunction@Learner.SupervisedLearner.LinearFeatureFunctionMLLearner(inputData, outputData, weighting);
            newMean = mean(obj.functionApproximator.getExpectation(size(inputData,1), inputData));
            
            oldCov =obj.functionApproximator.getCovariance;

            sumW    = sum(weighting);
            weighting = weighting / sumW;
            
            Z = ( sum(weighting)^2 - sum(weighting.^2));
            priorCov = obj.priorCov;
            priorCovWeight = obj.priorCovWeight;
            rangeOutput = obj.dataManager.getRange(obj.functionApproximator.outputVariable);
                
            if(Z > 0 )
                expectedOutput = obj.functionApproximator.getExpectation(size(outputData,1), inputData);
               
                
                difference =  expectedOutput - outputData;
                
                SigmaA = bsxfun(@times, difference, weighting)' * difference;
                SigmaA = 1 / Z * SigmaA;
                
                maxCorr = obj.maxCorr;
                minCov = obj.minCov .* rangeOutput;                                               
                
                numEffectiveSamples = sum(weighting) / max(weighting);
                %numEffectiveSamples  = sum(weighting)^2 / sum(weighting.^2);
                
%                 if(numEffectiveSamples<70)
%                     
%                     numEffectiveSamples
%                     display('low number of effective samples');
%                     
%                 end
               SigmaA = Learner.SupervisedLearner.boundCovariance(SigmaA, minCov, maxCorr);
                [~, cholA] = Learner.SupervisedLearner.regularizeCovariance(SigmaA,  diag(rangeOutput) * ...
                    priorCov, numEffectiveSamples,  priorCovWeight);
                
                %cholA = chol(SigmaA);
                obj.functionApproximator.setSigma(cholA);
                newCov = obj.functionApproximator.getCovariance;
                obj.KL = 1/2 *(trace(oldCov \ newCov ) + (newMean-oldMean)*(oldCov \ (newMean-oldMean)') ...
                    -size(outputData,2) +  sum(log(eig(oldCov))) - sum(log(eig(newCov))));
            else
                display('we are in nowhere');
                SigmaA = diag(rangeOutput) * priorCov * priorCovWeight / (1 + priorCovWeight);
                obj.functionApproximator.setCovariance(SigmaA);
            end
        end
    end
    
end
