classdef ProMPCombiWSpace < Data.DataManipulator & ...
        Functions.Mapping & ...
        Distributions.TrajectoryDistribution
    
    properties
        ProMPs;
        numJoints;
        
        ProMPOut;
    end
    
    methods
        
        function obj = ProMPCombiWSpace( dataManager, ProMPs )
            
            obj = obj@Distributions.TrajectoryDistribution();
            obj = obj@Functions.Mapping(dataManager, 'referenceState', {'basis','basisD','context','actvFactor'});
            
            obj.ProMPs = ProMPs;
            obj.numJoints = ProMPs{1}.numJoints;
            obj.registerMappingInterfaceDistribution();
            
            obj.addDataManipulationFunction('getStateDistribution', {'basis', 'basisD', 'context', 'actvFactor'}, ...
                {'mu_t', 'Sigma_t'}, Data.DataFunctionType.PER_EPISODE );
            
            obj.addDataManipulationFunction('getStateDistributionD', {'basis', 'basisD', 'basisDD', 'context', 'actvFactor', 'actvFactorD'}, ...
                {'mu_td', 'Sigma_td_half'}, Data.DataFunctionType.PER_EPISODE );
            
            obj.ProMPOut = TrajectoryGenerators.ProMPs(dataManager, obj.numJoints);
        end
        
        %% Traj Dist Interface
        
        function [mean, sigma] = getExpectationAndSigma(obj, ~, basis, basisD, context, actvFactor)
            [mean, sigma] = obj.getStateDistribution(basis, basisD, context, actvFactor );
        end
        
        
        function combineProMPWSpace(obj, actvFactor)
            
            combThres = 0.01;
            
            Sigma = zeros(obj.ProMPs{1}.numBasis, obj.ProMPs{1}.numBasis);
            mu  = zeros(obj.ProMPs{1}.numBasis, 1);
            
            for i = 1:length(obj.ProMPs)
                if (actvFactor(1,i) > combThres)
                    mu_wi = obj.ProMPs{i}.distributionW.bias;
                    Sigma_wi = obj.ProMPs{i}.distributionW.getCovariance;
                    
                    Sigma_wi_inv = Sigma_wi / actvFactor(1,i);
                    
                    mu = mu + pinv(Sigma_wi_inv,1e-15) * mu_wi;
                    
                    Sigma = Sigma + Sigma_wi_inv^-1;
                end
            end
            
            mu_w = Sigma \ mu;
            Sigma_w = Sigma^-1;
            Sigma_w = (Sigma_w + Sigma_w')*0.5;
            
            Sigma_w = Learner.SupervisedLearner.boundCovariance(Sigma_w, ...
                            obj.settings.minCovWeights*ones(obj.ProMPs{1}.numBasis,1), ...
                            obj.settings.maxCorrWeights);
                        
            [~, cholW] = Learner.SupervisedLearner.regularizeCovariance(Sigma_w,  ...
                obj.settings.priorCovWeights, 100,  obj.settings.priorCovWeightWeights);
            
            obj.ProMPOut.distributionW.setBias(mu_w);
            obj.ProMPOut.distributionW.setSigma(cholW);
            
        end
        
        %% Returns the state distribution format [ pos vel ]!
        function [mu_t, Sigma_t] = getStateDistribution(obj, basis, basisD, context, actvFactor )
            
            obj.combineProMPWSpace(actvFactor);
            if (~exist('context','var'))
                context = [];
            end
            [mu_t, Sigma_t] = obj.ProMPOut.getStateDistribution(basis, basisD, context);
            
        end
        
        function [mu_td, Sigma_td_half] = getStateDistributionD (obj, basis, basisD, basisDD, context, actvFactor, actvFactorD)
            
            obj.combineProMPWSpace(actvFactor);
            
            if (~exist('context','var'))
                context = [];
            end
            [mu_td, Sigma_td_half]  = obj.ProMPOut.getStateDistributionD(basis, basisD, basisDD, context);
            
            
        end
        
        
        
        
        
        
    end
    
end
