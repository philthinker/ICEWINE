classdef PandaOne < PandaZero
    %PandaOne Generate joint/Cartesian trajectories and store demo data of
    %Franka Emika Panda robot.
    %
    %   Haopeng Hu
    %   2020.06.09
    %   All rights reserved
    
    properties (Access = public)
        
    end
    
    properties (Access = protected)
        % Constraint param.
        JointConstraint;                  % 5 x 7, Joint constraint
        TorqueConstraint;               % 2 x 7, Torque constraint
        CartesianConstraint;            % 3 x 3, Cartesian constraint
    end
    
    methods
        function obj = PandaOne(kModelEnable)
            %PandaOne Init. the Panda
            %   kModelEnable: boolean, true for the property kModel needed.
            %   (default: false)
            %   Maker sure P. Corke's Robotics toolbox (>10.04) is installed if
            %   kModelEnable is set true.
            if nargin < 1
                kModelEnable = false;
            end
            obj = obj@PandaZero(kModelEnable);
            % Joint constraint
            % q_max, q_min, dq_max, ddq_max, dddq_max
            obj.JointConstraint = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973; ...
                                             -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;...
                                             2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;...
                                             15, 7.5, 10, 12.5, 15, 20, 20;...
                                             7500, 3750, 5000, 6250, 7500, 10000, 10000];
             % Torque constraint
             % tau_max, dtau_max
             obj.TorqueConstraint = [87, 87, 87, 87, 12, 12, 12; ...
                                                1000, 1000, 1000, 1000, 1000, 1000, 1000];
              % Cartesian constraint
              % dp_max, ddp_max, dddp_max
              % Note that here we take m as unit
              obj.CartesianConstraint = [1.7000, 2.5000, 2.1750; ...
                                                    13.0000, 25.0000, 10.0000; ...
                                                    6500.0000, 12500.0000, 5000.0000];
        end 
    end
    
    methods (Access = public)
        % Interpolation
        [trajOut, NOut] = interpJP(obj,trajIn);
        [trajOut, NOut] = interpJV(obj,trajIn);
        [trajOut, NOut] = interpCP(obj,trajIn);
        [trajOut, NOut] = interpCV(obj,trajIn);
        % Constraint check
        [flag, error_id] = checkJP(obj, traj);
        [flag, error_id] = checkJV(obj, traj);
        [flag, error_id] = checkCP(obj, traj);
        [flag, error_id] = checkCV(obj, traj);
    end
    
    
    methods (Access = public, Hidden = true)
        % Hidden methods
        function exeJoint = jSparse(obj,exeJointIn,THD)
            %jSparse Deprecated method
            %   Never use it anymore.
            exeJoint = exeJointIn;
        end
        function exeJoint = jtraj(obj,routes,dT)
            %jtraj Deprecated method
            %   Never use it anymore.
            exeJoint = routes;
        end
    end

end

