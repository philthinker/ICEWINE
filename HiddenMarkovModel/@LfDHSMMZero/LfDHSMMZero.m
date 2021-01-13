classdef LfDHSMMZero < HSMMZero
    %LfDHSMMZero HSMM for LfD studies
    %   Initialization with dim. and num. of states. and time difference
    %   States of the HSMM is assumed to be Gaussian
    %
    %   Haopeng Hu
    %   2021.01.01 Happy New Year!
    %   All rights reserved.
    %
    %   Notations:
    %   |   D:  Dim. of data
    %   |   N:  Num. of data
    %   |   K:  Num. of kernels
    %   |   M: Num. of demos
    %
    %   Recommendation:
    %   1   obj = LfDHSMMZero(D,K);
    %   2   obj = obj.initHMM...
    %   3   obj = obj.initTrans...
    %   4   obj = obj.leanHMM...
    
    properties
        
    end
    
    methods
        function obj = LfDHSMMZero(D,K,dt)
            %LfDHSMMZero Initialization with dim. and num. of states. and
            %time difference
            %   D: Integer, dim. of states
            %   K: Integer, num. of Gaussian states
            %   dt: Scalar, the time difference
            obj = obj@HSMMZero(D,K);
            dt = max(1e-3,dt);
            obj.dt = dt;
        end
        %% Sequence generation
    end
    
    methods
        % Auxiliary functions
    end
end

