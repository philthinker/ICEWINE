classdef ProMPCombi < Data.DataManipulator & ...
                      Functions.Mapping & ...
                      Distributions.TrajectoryDistribution
    
    properties
        ProMPs;
        numJoints;
    end
    
    methods
        
        function obj = ProMPCombi( dataManager, ProMPs )
            
            obj = obj@Distributions.TrajectoryDistribution();
            obj = obj@Functions.Mapping(dataManager, 'referenceState', {'basis','basisD','context','actvFactor'}); 
            
            obj.ProMPs = ProMPs;
            obj.numJoints = ProMPs{1}.numJoints;
            obj.registerMappingInterfaceDistribution();
            
            obj.addDataManipulationFunction('getStateDistribution', {'basis', 'basisD', 'context', 'actvFactor'}, ...
                {'mu_t', 'Sigma_t'}, Data.DataFunctionType.PER_EPISODE );
            
            obj.addDataManipulationFunction('getStateDistributionD', {'basis', 'basisD', 'basisDD', 'context', 'actvFactor', 'actvFactorD'}, ...
                {'mu_td', 'Sigma_td_half'}, Data.DataFunctionType.PER_EPISODE );
        end
        
        %% Traj Dist Interface
        
        function [mean, sigma] = getExpectationAndSigma(obj, ~, basis, basisD, context, actvFactor)
            [mean, sigma] = obj.getStateDistribution(basis, basisD, context, actvFactor );
        end
        
        
        %% Returns the state distribution format [ pos vel ]!
        function [mu_t, Sigma_t] = getStateDistribution(obj, basis, basisD, context, actvFactor )
            
            combThres = 0.01;
            
            nSteps  = size(basis,1);
            
            Sigma = zeros(obj.numJoints * nSteps * 2, obj.numJoints * nSteps * 2);
            mu_x  = zeros(obj.numJoints * nSteps * 2, 1);
            
            for i = 1:length(obj.ProMPs)
                if (actvFactor(1,i) > combThres)
                    [mu_xi, Sigma_ti] = obj.ProMPs{i}.getStateDistribution (basis, basisD, context);
                    
                    Sigma_ti_inv = Sigma_ti / actvFactor(1,i);                    
                    
                    mu_x = mu_x + pinv(Sigma_ti_inv,1e-15) * mu_xi;
                    
                    Sigma = Sigma + Sigma_ti_inv^-1;
                end
            end
            
            mu_t = Sigma \ mu_x;
            Sigma_t = Sigma^-1;
%             [mu_x1, Sigma_t1] = obj.ProMPs{1}.getStateDistribution (basis, basisD, context);
%             [mu_x2, Sigma_t2] = obj.ProMPs{2}.getStateDistribution (basis, basisD, context);
%             
%             S = ( Sigma_t1 + Sigma_t2 );
%             Sigma_t = (Sigma_t1 / S) * Sigma_t2;
%             mu_t = (Sigma_t1 / S) * mu_x2 + (Sigma_t2 / S) * mu_x1;
            
        end
        
        function [mu_td, Sigma_td_half] = getStateDistributionD (obj, basis, basisD, basisDD, context, actvFactor, actvFactorD)

            combThres = 0.01;
            
            [mu_t, Sigma_t] = obj.getStateDistribution( basis, basisD, context, actvFactor );
            
            nSteps  = size(basis,1);
            mu_td = zeros(obj.numJoints * nSteps * 2, 1);
            Sigma_td_half = zeros(obj.numJoints * nSteps * 2, obj.numJoints * nSteps * 2);
            
            
            for i = 1:length(obj.ProMPs)
                if (actvFactor(i) > combThres)
                    [mu_t_i, Sigma_t_i] = obj.ProMPs{i}.getStateDistribution (basis, basisD, context);
                    [mu_td_i, Sigma_td_half_i] = obj.ProMPs{i}.getStateDistributionD (basis, basisD, basisDD, context);
                    
                    tmp = (actvFactor(i) * (Sigma_t_i \ Sigma_td_half_i) - 0.5 *eye(size(Sigma_t_i)) * actvFactorD(i) ) / Sigma_t_i;
                    Sigma_td_half = Sigma_td_half + tmp;
                    
                   tmp1 = ( - actvFactor(i) * (Sigma_t_i \ (Sigma_td_half_i+Sigma_td_half_i')) ...
                                           + eye(size(Sigma_t_i)) * actvFactorD(i) ) / Sigma_t_i;
                                
                    mu_td = mu_td + tmp1 * mu_t_i + actvFactor(i) * (Sigma_t_i \ mu_td_i);
                                   
                end
            end            
            
            Sigma_td_half = Sigma_t * Sigma_td_half * Sigma_t;
            
            mu_td = Sigma_t * mu_td + (Sigma_td_half+Sigma_td_half') / Sigma_t  * mu_t;   
            
            
%             [mu_x1, Sigma_t1] = obj.ProMPs{1}.getStateDistribution (basis, basisD, context);
%             [mu_td1, Sigma_td_half1] = obj.ProMPs{1}.getStateDistributionD (basis, basisD, basisDD, context);
%             [mu_x2, Sigma_t2] = obj.ProMPs{2}.getStateDistribution (basis, basisD, context);
%             [mu_td2, Sigma_td_half2] = obj.ProMPs{2}.getStateDistributionD (basis, basisD, basisDD, context);
%             
%             S = ( Sigma_t1 + Sigma_t2 );
%             Sigma_t = (Sigma_t1 / S) * Sigma_t2;
%             mu_t = (Sigma_t1 / S) * mu_x2 + (Sigma_t2 / S) * mu_x1;
            
                        
%             Sigma_td_half = ( (Sigma_t1 \ Sigma_td_half1)  ) / Sigma_t1 ...
%                 + ( (Sigma_t2 \ Sigma_td_half2)  ) / Sigma_t2;
%             Sigma_td_half = Sigma_t * Sigma_td_half * Sigma_t;

%             tmp1 = ( -(Sigma_t1 \ (Sigma_td_half1+Sigma_td_half1')) ) / Sigma_t1;
%             tmp2 = ( -(Sigma_t2 \ (Sigma_td_half2+Sigma_td_half2')) ) / Sigma_t2;
%             
%             mu_td = tmp1 * mu_x1 + (Sigma_t1 \ mu_td1)...
%                    +tmp2 * mu_x2 + (Sigma_t2 \ mu_td2);
%             
%             mu_td = Sigma_t * mu_td + (Sigma_td_half+Sigma_td_half') / Sigma_t  * mu_t;

% Sigma_t_inv = (inv(Sigma_t1) + inv(Sigma_t2));
% Sigma_td_half = (Sigma_t_inv \ ( (Sigma_t1 \ Sigma_td_half1) / Sigma_t1 ...
%                 + (Sigma_t2 \ Sigma_td_half2) /  Sigma_t2 )) / Sigma_t_inv;
% 
%             
% Sigma_td = (Sigma_t_inv \ ( (Sigma_t1 \ (Sigma_td_half1+Sigma_td_half1')) / Sigma_t1 ...
%                 + (Sigma_t2 \ (Sigma_td_half2+Sigma_td_half2')) /  Sigma_t2 )) / Sigma_t_inv;
% 
% m =  max(abs(Sigma_td),abs( Sigma_td_half+Sigma_td_half'));
% A = sum(sum(abs(Sigma_td-( Sigma_td_half+Sigma_td_half'))./m))
%             
%   tmp = -((Sigma_t1 \ ( Sigma_td_half1 + Sigma_td_half1' )) / Sigma_t1) * mu_x1 ...
%         + Sigma_t1 \ mu_td1 ...
%         -((Sigma_t2 \ ( Sigma_td_half2 + Sigma_td_half2' )) / Sigma_t2) * mu_x2 ...  
%         + Sigma_t2 \ mu_td2;
%     
%   mu_td =  Sigma_t_inv \ tmp + ...
%            (Sigma_td_half+Sigma_td_half') * ( Sigma_t1 \ mu_x1 + Sigma_t2 \ mu_x2 );


        end
        
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
            
            phase  = obj.ProMPs{1}.phaseGenerator.generatePhase();
            phaseD  = obj.ProMPs{1}.phaseGenerator.generatePhaseD();
            
            basis  = obj.ProMPs{1}.basisGenerator.generateBasis(phase);
            basisD = obj.ProMPs{1}.basisGenerator.generateBasisD(phase);
            basisD = bsxfun(@times, basisD, phaseD);
            
            mu_t = zeros(length(phase)*2);
            Sigma_t = zeros(length(phase)*2,2);
            
            for i=1:length(phase)
                [mu_t(i:(i+1)), Sigma_t(i:(i+1),i:(i+1))] = obj.getStateDistribution(basis(i,:), basisD(i,:), [], []);
            end
                        
            size_muT = size(mu_t,1)/2;
            idx = (1:size_muT)+plotVel*size_muT;
            
            mu_t = reshape(mu_t(idx),obj.ProMPs{1}.numTimeSteps,[]);
            
            std_t = sqrt(diag(Sigma_t));
            std_t = reshape(std_t(idx),obj.ProMPs{1}.numTimeSteps,[]);
            
            figureHandles = Plotter.PlotterData.plotMeanAndStd( mu_t, std_t, name, 1:obj.numJoints, figureHandles, lineProps);
            
        end
        
        
    end
    
end
