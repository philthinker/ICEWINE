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
    %   1   obj = LfDHSMMZero(DP,DD,K,dt,logFlag);
    %   2   obj = obj.initHMM...
    %   3   obj = obj.initTrans...
    %   4   obj = obj.leanHMM...
    %   5   [...] = obj.constructTraj...
    
    properties
        r;      % scalar, LQT/LQR factor.
    end
    
    methods
        function obj = LfDHSMMZero(DP, DD, K, dt, logFlag)
            %LfDHSMMZero Initialization with dim. and num. of states. and
            %time difference
            %   DP: Integer, DP, dim. of position
            %   DD: Integer, DD, order of derivation
            %   K: Integer, num. of Gaussian states
            %   dt: Scalar, the time difference of the demo data
            %   logFlag: Boolean, true for log normal duration distribution
            %   (default: false)
            if nargin < 5
                logFlag = false;
            end
            obj = obj@TrajHSMMZero(DP,DD,K, logFlag);
            dt = max(1e-3,dt);  % 1ms
            obj.dt = dt;
            obj.r = 1e-5;
        end
        
        %% Sequence generation related functions
        [StateID] = initialState(obj, currP,c);
        [traj,h,seq] = constructTraj_LQR1(obj,currP,N);
        [traj,h,seq] = constructTraj_LQR1Itera(obj,currP,N);
    end
    
    methods (Access = protected)
        % Auxiliary functions
    end
end

