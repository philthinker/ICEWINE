classdef ProMPs < TrajectoryGenerators.LinearTrajectoryGenerator ...
                 & Functions.Mapping ...
                 & Distributions.TrajectoryDistribution
                   

    properties        
        distributionW;
        
        % Are used from push/pop 
        oldCholA_w;
        oldBias_w;
    end
    
    methods (Static)
        
        function [obj] = createFromTrial(trial)
            
            if (isprop(trial, 'phaseGenerator') && ~isempty(trial.phaseGenerator))
                obj = TrajectoryGenerators.ProMPs(trial.dataManager, trial.numJoints, trial.phaseGenerator, trial.basisGenerator);
            else
                obj = TrajectoryGenerators.ProMPs(trial.dataManager, trial.numJoints);
            end
        end
    end
    
    methods
        
        function obj = ProMPs(dataManager, numJoints, phaseGenerator, basisGenerator, distributionW )
            
            if (~exist('phaseGenerator', 'var') || isempty(phaseGenerator))
                phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);
            end

            if (~exist('basisGenerator', 'var') || isempty(basisGenerator))
                basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedGaussianBasisGenerator(dataManager, phaseGenerator);
            end           
  
            dataManager.addDataAlias('context',{}); % TODO check if does not exists and empty

            
            obj = obj@TrajectoryGenerators.LinearTrajectoryGenerator(dataManager, numJoints, phaseGenerator, basisGenerator);            
            obj = obj@Distributions.TrajectoryDistribution();
            dataManager.addDataAlias('referenceState',{'referencePos', 'referenceVel'});
            obj = obj@Functions.Mapping(dataManager, 'referenceState', {'basis','basisD','context'});            
            
            if (~exist('distributionW', 'var') || isempty(distributionW))
                Common.Settings().setIfEmpty('initSigmaWeights', 10^-9);
                distributionW = Distributions.Gaussian.GaussianLinearInFeatures(dataManager,'Weights','context','GaussianProMPs');        
            end
            obj.distributionW = distributionW;
                       
            obj.registerTrajectoryFunction(); 
            obj.registerMappingInterfaceDistribution();
            obj.setCallType('sampleFromDistribution',Data.DataFunctionType.PER_EPISODE);

            
            obj.addDataManipulationFunction('getStateDistribution', {'basis', 'basisD', 'context'}, ...
                                                            {'mu_t', 'Sigma_t'}, Data.DataFunctionType.PER_EPISODE );
            
            obj.addDataManipulationFunction('getStateDistributionD', {'basis', 'basisD', 'basisDD', 'context'}, ...
                                                            {'mu_td', 'Sigma_td_half'}, Data.DataFunctionType.PER_EPISODE );
                                                        
            obj.addDataManipulationFunction('sampleInitState', {'basis', 'basisD'}, {'states'}, true, true);
        end
        
        function [obj] = initObject(obj)
            obj.distributionW.initObject();             
        end        
        
        %% Distributions.Distribution implementations
        function [refState] = sampleInitState(obj, numEl, varargin)
            basis1D   = varargin{1};
            basis1Dd  = varargin{2};  
            
            basis1D   = basis1D(1,:);
            basis1Dd   = basis1Dd(1,:);
            
            [refState] = obj.sampleFromDistribution(numEl, basis1D, basis1Dd );
        end
        
        function [refState] = sampleFromDistribution(obj, numEl, varargin)
            %TODO proper demuxing
            basis1D   = varargin{1};
            basis1Dd  = varargin{2};
            
            wVec = obj.distributionW.sampleFromDistribution(numEl)';
            
            refState = [obj.getBasisMatrix(basis1D);
                        obj.getBasisMatrix(basis1Dd)] * wVec;
                    
             x = zeros(size(refState));
             x(1:2:end,:) = refState(1:(size(refState,1)/2),:);
             x(2:2:end,:) = refState((size(refState,1)/2+1):end,:);
             
             refState = x';
                    
%             x = reshape( reshape( refState(:), [], 2*numEl)', [], numEl)
%             
%             len = obj.numTimeSteps*obj.numJoints;
%             refState = [ reshape(refState(1:len),[],obj.numJoints); ...
%                          reshape(refState((1:len)+len),[],obj.numJoints); ];
%             figure;plot(refState(1:obj.numTimeSteps,:));
%             figure;plot(refState((1:obj.numTimeSteps)+obj.numTimeSteps,:));
        end
        
        function [mean, sigma] = getExpectationAndSigma(obj, basis, basisD, context)
            [mean, sigma] = obj.getStateDistribution(basis, basisD, context );
        end
        
        function getDataProbabilities(~, varargin)
            error('ProMPs: getDataProbabilities to implement');
        end
        
    end
    
    methods(Access=protected)
        
        function [] = registerMappingInterfaceDistribution(obj)
            obj.registerMappingInterfaceDistribution@Distributions.TrajectoryDistribution();
            obj.addDataManipulationFunction('getExpectationAndSigma', {'basis', 'basisD','context'}, ...
                                              {'referenceMean','referenceStd'});
        end
    end
    
    methods
        
        %% Assume we use the same basis for all DoFs 
        
        function [Phi_t] = getBasisMatrix(obj, basis1D)
            
            numTimeSteps = size(basis1D,1);
            numBasis = size(basis1D,2);

            r = obj.numJoints * numTimeSteps;
            c = numBasis * obj.numJoints;
            
            Phi_t = zeros (r,c);
            
            for j = 1:obj.numJoints
                
                idx_i = (j-1) * numTimeSteps + 1:j * numTimeSteps;
                idx_j = (j-1) * numBasis + 1:j *  numBasis;
                
                Phi_t( idx_i, idx_j  ) = basis1D;
                
            end
        end
        
        %% Returns the state distribution format [ pos vel ]!        
        function [mu_t, Sigma_t] = getStateDistribution(obj, basis, basisD, context )
            
            w_mu = obj.distributionW.getExpectation( 1, context)';         
            w_cov = obj.distributionW.getCovariance;
            
            Phi_t  = [ obj.getBasisMatrix(basis); 
                       obj.getBasisMatrix(basisD) ];
            
            mu_t = Phi_t * w_mu;            
            Sigma_t = Phi_t * w_cov * Phi_t';
            
        end
        
        function [mu_td, Sigma_td_half] = getStateDistributionD (obj, basis, basisD, basisDD, context)

            w_mu = obj.distributionW.getExpectation( 1, context)';
            w_cov = obj.distributionW.getCovariance;
            
            Phi_t  = [ obj.getBasisMatrix(basis);
                       obj.getBasisMatrix(basisD) ];
                   
            Phi_td = [ obj.getBasisMatrix(basisD);
                       obj.getBasisMatrix(basisDD) ]; 

            mu_td = Phi_td * w_mu;    
            Sigma_td_half = Phi_td * w_cov * Phi_t';
            
        end
        
        %% Push and pop functionality %TODO (why cloning failed?)
        
        function push(obj)    
            
            obj.oldCholA_w{end+1} = obj.distributionW.cholA;
            obj.oldBias_w{end+1}  = obj.distributionW.bias;      
            
        end
        
        function pop(obj)
            
            obj.distributionW.setSigma(obj.oldCholA_w{end});
            obj.distributionW.setBias(obj.oldBias_w{end});
            
            obj.oldCholA_w(end) = [];
            obj.oldBias_w(end)  = [];
            
        end
        
        function [bias_w, cholA_w] = getParam(obj)
            
            bias_w  = obj.distributionW.bias;
            cholA_w = obj.distributionW.cholA;    
            
        end
        
        function setParam(obj, bias_w, cholA_w)
            
            obj.distributionW.setBias(bias_w);
            obj.distributionW.setSigma(cholA_w);            
            
        end
        
        %% Conditioning / Combination (add extra files)
        function conditionTrajectory(obj, timepoint, y, sigmaVector, mask)    
            
            phase  = obj.phaseGenerator.generatePhase();
            basis  = obj.basisGenerator.generateBasis(phase);
            basisD = obj.basisGenerator.generateBasisD(phase);
            
            featuesIdx = round(timepoint / phase(end) / obj.dt);
            
            basis  = basis(featuesIdx,:);
            basisD = basisD(featuesIdx,:);
            
            Phi_t  = [ obj.getBasisMatrix(basis);
                       obj.getBasisMatrix(basisD) ];
            
            if ( exist('mask','var') )
                Phi_t = Phi_t(logical(mask),:);
            end
            
            muW  = obj.distributionW.getMean();
            sigW = obj.distributionW.getCovariance();
            
            %Gaussian conditioning 
            tmp = sigW * Phi_t' / ( diag(sigmaVector) + Phi_t * sigW * Phi_t');
            
            newMuW  = muW + tmp * (y - Phi_t * muW);
            newSigW = sigW - tmp * Phi_t * sigW;
            
            r = eye(size(newSigW))*1;
            [~, cholA] = Learner.SupervisedLearner.regularizeCovariance(newSigW, r, 1,  1e-16);
            obj.distributionW.setSigma(cholA);
            obj.distributionW.setBias(newMuW);
            
        end  
        
        %% Redundant ploting... without the need of data TODO        
        function [figureHandles] = plotStateDistribution(obj, plotVel, figureHandles, lineProps)
            
            if ( ~exist('figureHandles','var') )
                figureHandles = [];
            end
            
            if ( ~exist('lineProps','var') )
                lineProps = [];
            end       
            
            name = 'DesiredPos';
            if ( exist('plotVel','var') && plotVel == 1 )
                name = 'DesiredVel';
            else
                plotVel = 0;
            end
            
            phase  = obj.phaseGenerator.generatePhase();
            phaseD  = obj.phaseGenerator.generatePhaseD();
            
            basis  = obj.basisGenerator.generateBasis(phase);
            basisD = obj.basisGenerator.generateBasisD(phase);
            basisD = bsxfun(@times, basisD, phaseD);
            [mu_t, Sigma_t] = obj.getExpectationAndSigma(basis, basisD, []);
                        
            size_muT = size(mu_t,1)/2;
            idx = (1:size_muT)+plotVel*size_muT;
            
            mu_t = reshape(mu_t(idx),obj.numTimeSteps,[]);
            
            std_t = sqrt(diag(Sigma_t));
            std_t = reshape(std_t(idx),obj.numTimeSteps,[]);
            
            figureHandles = Plotter.PlotterData.plotMeanAndStd( mu_t, std_t, name, 1:obj.numJoints, figureHandles, lineProps);
            
        end
        
        function [figureHandles] = plotAccDistribution(obj, figureHandles, lineProps)
            
            if ( ~exist('figureHandles','var') )
                figureHandles = [];
            end
            
            if ( ~exist('lineProps','var') )
                lineProps = [];
            end       
            
            name = 'DesiredAcc';
            
            phase  = obj.phaseGenerator.generatePhase();
            phaseD  = obj.phaseGenerator.generatePhaseD();
            phaseDD  = obj.phaseGenerator.generatePhaseDD();
            
%             basis  = obj.basisGenerator.generateBasis(phase);
            basisD = obj.basisGenerator.generateBasisD(phase);
            
            basisDD = obj.basisGenerator.generateBasisDD(phase);
            basisDD = bsxfun(@times, basisDD, phaseD.^2) + bsxfun(@times, basisD, phaseDD );            
%             basisD  = bsxfun(@times, basisD, phaseD);
            
            
            w_mu = obj.distributionW.getExpectation( 1, [])';
            w_cov = obj.distributionW.getCovariance;
            
            Phi_t  = obj.getBasisMatrix(basisDD);
            
            mu_t = Phi_t * w_mu;
            Sigma_t = Phi_t * w_cov * Phi_t';

            
            mu_t = reshape(mu_t,obj.numTimeSteps,[]);
            
            std_t = sqrt(diag(Sigma_t));
            std_t = reshape(std_t,obj.numTimeSteps,[]);
            
            figureHandles = Plotter.PlotterData.plotMeanAndStd( mu_t, std_t, name, 1:obj.numJoints, figureHandles, lineProps);
            
        end
        
      
    end
 
end
