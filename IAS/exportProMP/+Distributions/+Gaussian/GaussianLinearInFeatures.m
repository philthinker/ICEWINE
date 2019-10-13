classdef GaussianLinearInFeatures < Functions.FunctionLinearInFeatures & Distributions.DistributionWithMeanAndVariance & ParametricModels.ParametricModel
    % The  GaussianLinearInFeatures class models gaussian distributions where the mean can 
    % be a linear function of the feature vectors. 
    %
    % This class models a linear Gaussian distribution in the form of 
    % \f$\mathcal{N}(\boldsymbol{y}| \boldsymbol{b} + \boldsymbol{W} \boldsymbol{\phi}(\boldsymbol{x}), \boldsymbol{\Sigma})\f$ we omit the scaling scalar 
    % because we will work with normalized data.
    %
    % This class is a subclass of <tt>Distributions.DistributionWithMeanAndVariance</tt>
    % and defines the abstract <tt>getExpectationAndSigma</tt> so this class
    % acts like a Gaussian distribution in the functions
    % <tt>sampleFromDistribution</tt> and <tt>getDataProbabilities</tt> of the superclass.
    %
    % Because of its numerical features this class will work via cholesky
    % matrix internally, but is able to return characteristics like mean,
    % expectation and the sigma matrix like expected.
    %
    % see Functions.Mapping for more information how to use outputVariable
    % and inputVariables etc
    properties (SetAccess = protected)
        cholA % Matrix in the Cholesky decomposition
        indexForCov
        covMat
    end
    
    properties
        saveCovariance = false;
    end
    
    properties (SetObservable, AbortSet)
        initSigma = 0.1;
    end
    
    methods
        
        function obj = GaussianLinearInFeatures(dataManager, outputVariable, inputVariables, functionName, varargin)
            % @param dataManager Data.DatManager to operate on
            % @param outputVariable set of output Variables of the gaussian function
            % @param inputVariables set of input Variables of the gaussian function
            % @param functionName name of the gaussian function
            % @param varargin optional featureGenerator, doInitWeights (see superclass Functions.FunctionLinearInFeatures)
            superargs = {};
            if (nargin >= 1)
                superargs = {dataManager, outputVariable, inputVariables, functionName, varargin{:}};
            end
            
            obj = obj@ParametricModels.ParametricModel();            
            obj = obj@Functions.FunctionLinearInFeatures(superargs{:});
            obj = obj@Distributions.DistributionWithMeanAndVariance();
            
            
            if (nargin >= 1)
                if (ischar(outputVariable))
                    obj.linkProperty('initSigma', ['initSigma',  upper(obj.outputVariable(1)), obj.outputVariable(2:end)]);
                    
                else
                    obj.linkProperty('initSigma');
                end
            end
            obj.registerMappingInterfaceDistribution();
            obj.registerMappingInterfaceFunction();
            obj.registerGradientModelFunction();
        end
        
        function [numParameters] = getNumParameters(obj)
            numParameters = obj.getNumParameters@Function.LinearInFeatures() + obj.numParameters + obj.dimOutput * (obj.dimOutput + 1) / 2;
        end
        
        function [] = initObject(obj)
            
            obj.initObject@Functions.FunctionLinearInFeatures();
            
            initSigma = obj.initSigma;
            
            range = obj.dataManager.getRange(obj.outputVariable);
           
            obj.cholA = diag(range .* initSigma);
            
            
            obj.indexForCov = [];
            index = 0;
            for i = 1:obj.dimOutput
                obj.indexForCov = [obj.indexForCov, index + (i:obj.dimOutput)];
                index = index + obj.dimOutput;
            end
        end
                        
        
        function [mean, sigma] = getExpectationAndSigma(obj, numElements, varargin)
            mean = obj.getExpectation(numElements, varargin{:});
            
            sigma(1,:, :) = obj.cholA;
        end
        
        function [covariance] = getCovariance(obj)
            % returns covariance matrix
            if obj.saveCovariance
                covariance = obj.covMat;
            else
                covariance = obj.cholA' * obj.cholA;
            end
        end
        
        function [bias] = getMean(obj)
            % returns the bias vector
            bias = obj.bias;
        end
        
        
        function [] = setCovariance(obj, covMat)
            if obj.saveCovariance
                obj.covMat = covMat;
            else
                obj.cholA = chol(covMat);
            end
        end
        
        function [] = setSigma(obj, cholA)
            % expects the squareroot of the Covariance in cholesky form
            if obj.saveCovariance
                obj.cov = cholA' * cholA;
            else
                obj.cholA = cholA;
            end
        end
        
        function sigma = getSigma(obj)
            if obj.saveCovariance
                sigma = chol(obj.covMat);
            else
                sigma = obj.cholA;
            end
        end
        
        function [muNew, SigmaNew] = getJointGaussians(obj, muInput, SigmaInput)
            muNew = [muInput; obj.bias + obj.weights*muInput];
            tmp = (obj.weights*SigmaInput)';
            SigmaNew = [SigmaInput tmp; tmp' (obj.getCovariance() + obj.weights*tmp)];
        end
        
        function [] = getGaussianFromJoint(obj, muJoint, SigmaJoint)
            
            tmpN = obj.dimInput;
            muInput = muJoint(1:tmpN);
            SigmaInput = SigmaJoint(1:tmpN,1:tmpN);
            
            
            SigmaInputOutput = SigmaJoint(1:tmpN,(tmpN+1):end);
            obj.weights = SigmaInputOutput'/SigmaInput;
            obj.bias = muJoint((tmpN+1):end)-obj.weights*muInput;
            
            SigmaOutput = SigmaJoint((tmpN+1):end,(tmpN+1):end)-obj.weights*SigmaInputOutput;
            obj.setCovariance(SigmaOutput);
        end
        
        function [gradient] = getLikelihoodGradient(obj, inputMatrix, outputMatrix)
            
            expectation = obj.getExpectation(size(inputMatrix,1), inputMatrix);
            gradientFunction = obj.getGradient(inputMatrix);
            
            [n, d] = size(outputMatrix);
            zmx = outputMatrix - expectation;
            C  = obj.getCovariance();
                        
            duplicate = size(inputMatrix,2) + 1;
            gradMeanFactor = zmx/C;
            gradMeanFactor = repmat(gradMeanFactor, 1, duplicate);
            gradMean = gradientFunction .* gradMeanFactor;
            
            gradCholA = zeros(n, d*(d+1)/2);
            
            for s = 1:n
                R = obj.cholA'\(zmx(s, :)'*zmx(s, :))/C - diag(diag(obj.cholA).^-1);
                gradCholA(s,:) = R(obj.indexForCov);
            end
            
            gradient = [gradMean, gradCholA];
        end
        
        function Fim = getFisherInformationMatrix(obj)
            
            noPars = obj.numParameters;
            d = obj.dimOutput;
            
            Fim = zeros(noPars, noPars);
            C = obj.getCovariance();
            F0 = inv(C);
            Fim(1:d, 1:d) = F0;
            
            ix_act = d+1;
            ix_nxt = 2*d;
            
            %TODO Check with Paper
            for k = 1:d
                
                f = zeros(d-k+1, d-k+1);
                f(1, 1) = obj.cholA(k, k)^-2;
                D = F0(k:end, k:end);
                
                f = f + D;
                
                Fim(ix_act:ix_nxt, ix_act:ix_nxt) = f;
                
                dummy = ix_act;
                ix_act = ix_nxt + 1;
                ix_nxt = ix_nxt + (ix_nxt - dummy);
                
            end            
        end
        
        
        function [] = setParameterVector(obj, theta)
            obj.setParameterVector@Functions.FunctionLinearInFeatures(theta);
            theta = theta(obj.dimOutput * (1 + obj.dimInput) + 1 : end);
            obj.cholA(obj.indexForCov) = theta;
        end
        
        function [theta] = getParameterVector(obj)
            theta = obj.getParameterVector@Functions.FunctionLinearInFeatures();
            theta = [theta, obj.cholA(obj.indexForCov)];
        end
    end
end
