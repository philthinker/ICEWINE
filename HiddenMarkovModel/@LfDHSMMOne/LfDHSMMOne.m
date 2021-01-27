classdef LfDHSMMOne < HSMMZero
    %LfDHSMMOne Hidden semi-Markov Model for LfD usage
    %   - Traj-HSMM setting is available.
    %   - Gaussian emission probability.
    %   - Functions on Sd manifold are NOT available so far.
    %   Initialize it with K, DP, and DD.
    %   Just enjoy it!
    %   
    %   Haopeng Hu
    %   2021.01.25
    %   All rights reserved
    %
    %   Notations:
    %   |   D:  Dim. of data (DP * DD)
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
        DP;       % Integer, dim. of the position data
        DD;       % Integer, ord. of differentiation
        r;           % Scalar, LQT/LQR factor.
%         A;          % A matrix of the dynamics
%         B;          % B matrix of the dynamics
    end
    
    methods
        function obj = LfDHSMMOne(K, DP, DD)
            %LfDHSMMOne Initialize it with K, DP, and DD.
            %   K: Integer >= 1, constant K
            %   DP: Integer >= 2, constant DP
            %   DD: Integer >= 1, constant DD
            K = max([round(K),1]);
            DP = max([round(DP),2]);
            DD = max([round(DD),1]);
            obj = obj@HSMMZero(DP,K,false); % We only use  position as state
            obj.DP = DP; obj.DD = DD;
            obj.dt = 1e-2;
            obj.r = 1e-3;
%             % Dynamics
%             if DD > 1
%                 A1d = zeros(DD);
%                 for i=0:DD-1
%                     A1d = A1d + diag(ones(obj.DD-i,1),i) * obj.dt^i * 1/factorial(i); %Discrete 1D
%                 end
%                 B1d = zeros(obj.DD,1);
%                 for i=1:obj.DD
%                     B1d(obj.DD-i+1) = obj.dt^i * 1/factorial(i); %Discrete 1D
%                 end
%                 obj.A = kron(A1d, eye(obj.DP)); %Discrete nD
%                 obj.B = kron(B1d, eye(obj.DP)); %Discrete nD
%             else
%                 obj.A = eye(DP);
%                 obj.B = zeros(DP,1);
%             end
        end
        %% Trajectory generation
        [traj] = SampleHSMMLQR_SC(obj, p0, N);  % LQT given N
        [traj,u] = LQTBath(obj,p0,seq);   % LQT in bath manner
        [traj,u] = LQTIterative(obj,p0,seq);  % LQT in interative manner
    end
    
    methods (Access = public)
        % Auxiliary functions
        [SID] = stateDetermine(obj, p); % Determining the related state
        function [obj] = resetStatePrior(obj, SID)
            %resetStatePrior Reset the StatePrior property to be 1 for
            %state SID.
            %   SID: Integer, the state ID
            SID = max([round(SID),1]);
            obj.StatePrior = zeros(obj.K,1);
            obj.StatePrior(SID) = 1.0;
        end
    end
end

