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
        
        function [flag, error_id] = checkTraj(obj, traj, mode)
            %checkTraj Check the trajectory to make sure it violates no
            %constraint
            %   traj: N x 7 or 4 x 4 x N or N x 16 or N x 6, trajectory data
            %   |   N x 7: joint position/velocity trajectory
            %   |   4 x 4 x N: Cartesian position trajectory
            %   |   N x 16: Cartesian position trajectory
            %   |   N x 6: Cartesian velocity trajectory
            %   mode: integer, trajectory mode
            %   |   0: joint position trajectory
            %   |   1: joint velocity trajectory
            %   |   2: Cartesian position trajectory
            %   |   3: Cartesian velocity trajectory
            %   flag: boolean, true for no violation
            %   error_id: integer, the violation item (No violation: 0)
            flag = false;
            error_id = 0;
            % Check the trajectory's dimension and interval
            if mode == 0
                % Joint position
                if size(traj,2) == 7
                    [flag,error_id] = obj.checkJP(traj);
                else
                    error_id = 5;
                end
            elseif mode == 1
                % Joint velocity
                if size(traj,2) == 7
                    [flag,error_id] = obj.checkJV(traj);
                end
            elseif mode == 2
                % Cartesian position
                if ndims(traj) == 3
                    % SE(3) trajectory
                    if size(traj,1) == 4 && size(traj,2) == 4
                        [flag, error_id] = obj.checkCP(traj);
                    end
                elseif ismatrix(traj)
                    % SE(3) trajectory of flattened form
                    if size(traj,2) == 16
                        % Flatten to SE(3) form
                        traj = fold2SE3(traj);
                        [flag, error_id] = obj.checkCP(traj);
                    end
                end
            elseif mode == 3
                % Cartesian velocity
                if size(traj,2) == 6
                    [flag,error_id] = obj.checkCV(traj);
                end
            end
        end
        
        function [trajOut, NOut, error_id] = interpolate(obj, trajIn, mode)
            %interpolate Interpolate a trajectory given some route points
            %   trajIn: N x 7 or 4 x 4 x N or N x 16 or N x 6, trajectory data
            %   |   N x 7: joint position/velocity trajectory
            %   |   4 x 4 x N: Cartesian position trajectory
            %   |   N x 16: Cartesian position trajectory
            %   |   N x 6: Cartesian velocity trajectory
            %   mode: integer, trajectory mode
            %   |   0: joint position trajectory
            %   |   1: joint velocity trajectory
            %   |   2: Cartesian position trajectory
            %   |   3: Cartesian velocity trajectory
            %   |   4: Catesian position trajectory but for a flattened
            %   trajectory out
            %   trajOut: NOut x 7 or 4 x 4 x NOut or NOut x 16, trajectory
            %   out
            %   NOut: integer, number of data points in trajectory out
            %   error_id: integer, 0 for no error
            error_id = 0;
            trajOut = [];
            NOut = 0;
            % Check the trajectory's dimension and interval
            if mode == 0
                % Joint position
                if size(trajIn,2) == 7 && size(trajIn,1) > 1
                    [trajOut, NOut] = obj.interpJP(trajIn);
                else
                    error_id = 1;
                end
            elseif mode == 1
                % Joint velocity
                if size(trajIn,2) == 7 && size(trajIn,1) > 1
                    [trajOut, NOut] = obj.interpJV(trajIn);
                else
                    error_id = 1;
                end
            elseif mode == 2 || mode == 4
                % Cartesian position
                if ndims(trajIn) == 3
                    % SE(3) trajectory
                    if size(trajIn,1) == 4 && size(trajIn,2) == 4 && size(trajIn,3) > 1
                        [trajOut,NOut] = obj.interpCP(trajIn);
                        if mode == 4
                            % Flatten
                            trajOut = flattenSE3(trajOut);
                        end
                    else
                        error_id = 1;
                    end
                elseif ismatrix(trajIn)
                    % SE(3) trajectory of flattened form
                    if size(trajIn,2) == 16
                        % Flatten to SE(3) form
                        trajIn = fold2SE3(trajIn);
                        [trajOut, NOut] = obj.interpCP(trajIn);
                        if mode == 4
                            % Flatten
                            trajOut = flattenSE3(trajOut);
                        end
                    else
                        error_id = 1;
                    end
                else
                    error_id = 1;
                end
            elseif mode == 3
                % Cartesian velocity
                if size(trajIn,2) == 6
                    [trajOut, NOut] = obj.interpCV(trajIn);
                end
            end
        end
    end
    
    methods (Access = protected)
        % Constraint check
        [flag, error_id] = checkJP(obj, traj);
        [flag, error_id] = checkJV(obj, traj);
        [flag, error_id] = checkCP(obj, traj);
        [flag, error_id] = checkCV(obj, traj);
        % Interpolation
        [trajOut, NOut] = interpJP(obj,trajIn);
        [trajOut, NOut] = interpJV(obj,trajIn);
        [trajOut, NOut] = interpCP(obj,trajIn);
        [trajOut, NOut] = interpCV(obj,trajIn);
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

