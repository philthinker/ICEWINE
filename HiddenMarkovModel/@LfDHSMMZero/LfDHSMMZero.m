classdef LfDHSMMZero < TrajHSMMZero
    %LfDHSMMZero HSMM for LfD studies
    %   Initialization with dim. and num. of states. and time difference
    %   States of the HSMM is assumed to be Gaussian
    %
    %   Haopeng Hu
    %   2021.01.14
    %   All rights reserved.
    %
    %   Notations:
    %   |   D:  Dim. of data
    %   |   DP: Dim. of position
    %   |   DD: Order of derivation
    %   |   N:  Num. of data
    %   |   K:  Num. of kernels
    %   |   M: Num. of demos
    %
    %   Recommendation:
    %   1   obj = LfDHSMMZero(D,K);
    %   2   obj = obj.initHMM...
    %   3   obj = obj.initTrans...
    %   4   obj = obj.leanHMM...
    %   5   [...] = obj.reconstructStSeq_...
    %   6   [...] = obj.constructTraj...
    
    properties
        
    end
    
    methods
        function obj = LfDHSMMZero(DP, DD, K,dt)
            %LfDHSMMZero Initialization with dim. and num. of states. and
            %time difference
            %   DP: Integer, DP, dim. of position
            %   DD: Integer, DD, order of derivation
            %   K: Integer, num. of Gaussian states
            %   dt: Scalar, the time difference
            obj = obj@TrajHSMMZero(DP,DD,K);
            dt = max(1e-3,dt);  % 1ms
            obj.dt = dt;
        end
        
        %% Sequence generation related functions
        [StateID] = initialState(obj, currP,c);
        [traj,trajSigma,h,seq] = constructTraj_AdaptInit0(obj, currP,N);
        [traj,trajSigma,h,seq] = constructTraj_TP0(obj, initP, goalP, dt);
    end
    
    methods (Access = protected)
        % Auxiliary functions
    end
end

