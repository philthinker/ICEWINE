classdef TrajectoryDistribution  < Distributions.DistributionWithMeanAndVariance
    
    methods  
        
        function obj = TrajectoryDistribution()
            obj = obj@Distributions.DistributionWithMeanAndVariance();
        end
        
        %% Plotting
        function [figureHandles] = plotStateDistribution(obj, data, plotVel, figureHandles, lineProps)
            
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
            
            numTimeSteps = length(data.getDataEntry('jointPositions',1,:));
            
            [mu_t, Sigma_t] = obj.callDataFunctionOutput('getExpectationAndSigma', data,1,:);
            
            size_muT = size(mu_t,1)/2;
            idx = (1:size_muT)+plotVel*size_muT;
            
            mu_t = reshape(mu_t(idx),numTimeSteps,[]);
            
            std_t = sqrt(diag(Sigma_t));
            std_t = reshape(std_t(idx),numTimeSteps,[]);
            
            figureHandles = Plotter.PlotterData.plotMeanAndStd( mu_t, std_t, name, 1:obj.numJoints, figureHandles, lineProps);
            
        end
        
    end
    
end