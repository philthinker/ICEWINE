classdef TrajGMMOne < TPGMMOne
    %TrajGMMOne Trajectory-Gaussian mixture model
    %   It provides a reproduction solution besides GMR. Task-parameterized
    %   setting is also supported.
    %   Init. it with K, D, DD, F. For efficiency, when F == 1 we use
    %   obj.Mu instead of obj.Mus, so do obj.Sigma and obj.Sigmas.
    %   Notations:
    %   |   N: num. of data
    %   |   M: num. of demos
    %   |   K: num. of Gaussian distri.
    %   |   F: nun. of frames
    %   |   D: dim. of data
    %   |   DD: ord. of diff. (1 for pos., 2 for vel. and 3 for acc.)
    %   |   DPos: dim. of the pos. data
    %   |   TPDemo struct:
    %   |   |   data: D x N, demo data
    %   |   |   A: D x D x F, orientation matrices
    %   |   |   b: D x F, position vectors
    %   |   |   TPData: D x F x N, demo data in each frame
    %
    %   Haopeng Hu
    %   2020.03.07
    %   All rights reserved
    %
    
    properties (Access = public)
        nDiff;      % ord. of diff., DD
    end
    
    properties (Access = protected)
        tpFlag;     % boolean, true for TP-Traj-GMM
    end
    
    methods
        function obj = TrajGMMOne(nKernel,nPosVar,nDiff,nFrame)
            %TrajGMMOne Init. the Traj-GMM with K, DPos, DD, F.
            %   nKernel: integer, K
            %   nPosVar: integer, dim. of pos. var., D/DD
            %   nDiff: integer, DD
            %   nFrame: integer, F
            K = round(nKernel(1,1)); DPos = round(nPosVar(1,1));
            DD = round(nDiff(1,1)); F = round(nFrame(1,1));
            obj = obj@TPGMMOne(K, DPos * DD, F);
            obj.tpFlag = F > 1;
            obj.nDiff = DD;
        end
        
        function obj = learnGMM(obj,Demos)
            %learnGMM 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj = obj.Property1 + Demos;
        end
        
        function obj = reproduct(obj,query,LSMode)
            %learnGMM 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj = obj.Property1 + query;
        end
    end
    
    methods (Access = public)
        % Auxiliary func.
        [dynaData] = computeDynamicData(obj,data,dt,nDiff);
    end
    
    methods (Access = protected)
        [Phi,Phi1,Phi0] = constructPhi(obj,N,DD,DPos,M,dt);
    end
end

