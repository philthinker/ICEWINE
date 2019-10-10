classdef GMMRobio2019 < GMMZero
    %GMMRobio2019 The correctivable Gaussian Mixture Model used for the
    %Dali trip in 2019. It serves to managet the correction data and plot
    %the figures. It is NOT recommended to learn this object directly.
    %   Haopeng Hu
    %   2019.09.30
    %   All rights reserved
    
    properties (Access = public)
        corrections;
        exeJointPlus;
    end
    
    properties (Access = public)
        NExeJointPlus;
    end
    
    methods
        function obj = GMMRobio2019(GMM)
            %GMMRobio2019 It is built by another GMM
            %   GMM: 1 x 1 GMMZero
            obj = obj@GMMZero(GMM.nKernel,GMM.nVar,GMM.dt);
            obj.Mu = GMM.Mu;
            obj.Sigma = GMM.Sigma;
            obj.Prior = GMM.Prior;
            obj.corrections = [];
            obj.exeJointPlus = cell(1,15);
            obj.NExeJointPlus = 0;
        end
        
        function [obj,query] = addJCorrection(obj,queryIndex,exeJointPlus,correcJoint)
            %addJCorrection Add a correction in joint space
            %   queryIndex: positive scalar, the query index where the
            %   correction is performed.
            %   exeJointPlus: N x 8, the executing trajectory whose first
            %   column is the time series.
            %   correcJoint: 1 x 8, the correction Mu
            correcJfina = correcJoint(end,:);
            addSigma = eye(obj.nVar).*1e-5;
            addSigma(1,1) = 0.0016;
            
            % Find the corresponded query value (Minimal distance)
            query = exeJointPlus(queryIndex,1);
            addMu = [query, correcJfina];
            obj.corrections = [obj.corrections;addMu];
            obj.Mu = [obj.Mu; addMu];
            tmpSigma = repmat(eye(obj.nVar),[1,1,obj.nKernel+1]);
            tmpSigma(:,:,1:obj.nKernel) = obj.Sigma;
            tmpSigma(:,:,end) = addSigma;
            obj.Sigma = tmpSigma;
            
            % Regulate Priors 
            if query >= obj.Mu(obj.nKernel,1)
                % Latter than the last one
                addPrior = obj.Prior(obj.nKernel) * 1.8;
            elseif query <= obj.Mu(1,1)
                % Former than the first one
                addPrior = obj.Prior(1) * 1.8;
            else
                for i = 1:obj.nKernel-1
                    if query >= obj.Mu(i,1) && query <= obj.Mu(i+1,1)
                        addPrior = max([obj.Prior(i),obj.Prior(i+1)]) * 1.8;
                        break;
                    end
                end
            end
            obj.Prior = probNormalize([obj.Prior;addPrior]);
            
            obj.nKernel = obj.nKernel + 1;
%             obj = obj.sortMu(1);
            
            % The exeJointPlus is stored in the object
            obj.NExeJointPlus = obj.NExeJointPlus + 1;
            obj.exeJointPlus{obj.NExeJointPlus} = exeJointPlus;
        end
        
        function gmrJoint = plotComparison(obj)
            %plotComparison Plot the original and new trajectories
            t = obj.exeJointPlus{1}(:,1);
            [gmrJoint,~] = obj.GMR(t);
            figure;
            for i = 1:obj.nVar-1
                subplot(obj.nVar-1,1,i);
                for j = 1:obj.NExeJointPlus
                    plot(t,obj.exeJointPlus{j}(:,i+1),'Color',[0.5,0.5,0.5]);
                    hold on;
                end
                plot(t,gmrJoint(:,i),'Color',[0.63,0.13,0.94]); % It's purple
                grid on;    ylabel(strcat('Joint',int2str(i)));
                axis([t(1),t(end),-inf,inf]);
            end
        end
        
        function [] = plotAddedModel(obj,demos)
            %plotAddedModel
            figure;
            M = length(demos);
            D = size(demos{1},2)-1; % The very left column is the time series
            
            for i = 1:D
                subplot(D,1,i);
                for j = 1:M
                    % Plot the demos
                    plot(demos{j}(:,1),demos{j}(:,i+1),'Color',[0.45,0.45,0.45]);
                    hold on;
                end
                tmpSigma = obj.Sigma([1,i+1],[1,i+1],:);
                % Transparency
                valAlpha = 0.5;
                color1 = [1,0,0];
                tmpIndex = (1:obj.nKernel-obj.NExeJointPlus);
                obj.plotGMM2SCPro(obj.Mu(tmpIndex,[1 i+1]),tmpSigma(:,:,tmpIndex),color1,valAlpha);
                color2 = [0.63,0.13,0.94];
                tmpIndex = (obj.nKernel-obj.NExeJointPlus+1:obj.nKernel);
                obj.plotGMM2SCPro(obj.Mu(tmpIndex,[1 i+1]),tmpSigma(:,:,tmpIndex),color2,valAlpha);
                grid on;
                axis([demos{1}(1,1), demos{1}(end,1), -inf,inf]);
            end
        end
    end
end

