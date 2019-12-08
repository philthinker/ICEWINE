classdef GMMSake < GMMOne
    %GMMSake Gaussian mixture model for *Sake* temporal alignment method
    %   nKernel: integer, K
    %   nVar: integer, D
    
    %   Haopeng Hu
    %   2019.02.07
    %   All rights reserved
    
    properties
        
    end
    
    methods
        function obj = GMMSake(nKernel,nVar)
            %GMMSake 
            %   nKernel: integer, K
            %   nVar: integer, D
            obj = obj@GMMOne(nKernel,nVar);
        end
        
        function obj = initGMMKMeans(obj,panda)
            %initGMMKMeans Init. the GMM by K-Means algorithm, only the
            %positions are considered
            %   panda: 1 x 1, PandaZero object
            Demos = cell(1,panda.NCartesianDemo);
            for i = 1:panda.NCartesianDemo
                Demos{i} = permute(panda.demoCartesian{i}(1:3,4,:),[1,3,2]);
            end
            Data = obj.demoFlatten(Demos);
            obj = initGMMKMeans@GMMOne(obj,Data);
        end
        
        function obj = learnGMMEM(obj,panda)
            %learnGMMEM Learn the GMM by EM algorithm, only the positions
            %are considered
            %   panda: 1 x 1, PandaZero object
            Demos = cell(1,panda.NCartesianDemo);
            for i = 1:panda.NCartesianDemo
                Demos{i} = permute(panda.demoCartesian{i}(1:3,4,:),[1,3,2]);
            end
            Data = obj.demoFlatten(Demos);
            obj = obj.learnGMM(Data);
        end
        
        function [] = plotPandaDemos(obj,panda)
            %plotPandaDemos Plot the demos of panda
            %   panda: 1 x 1 PandaZero object
            for i = 1:panda.NCartesianDemo
                tmpTraj = permute(panda.demoCartesian{i}(1:3,4,:),[1,3,2]);
                plot3(tmpTraj(1,:),tmpTraj(2,:),tmpTraj(3,:));
                hold on;
            end
            grid on;
            axis equal;
        end
        
    end
    
    methods (Access = protected)
        Data = demoFlatten(obj,demos);
    end
end

