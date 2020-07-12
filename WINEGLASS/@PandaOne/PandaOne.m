classdef PandaOne < PandaZero
    %PandaOne Generate joint/Cartesian trajectories and store demo data of
    %Franka Emika Panda robot.
    %
    %   Haopeng Hu
    %   2020.06.09
    %   All rights reserved
    
    properties (Access = public)
        demoPQuaternion;            % 1 x NDemo cell, Demos of position and unit quaternions
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
            % Quaternion
            obj.demoPQuaternion = cell(1,1); % It won't be init.
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
        function obj = addCartesianDemoPQ(obj,demo)
            %addCartesianDemoPQ Add Cartesian demo by [x y z w qx qy qz]
            %   demo: N x 7, [x y z w qx qy qz]
            obj.NCartesianDemo = obj.NCartesianDemo + 1;
            obj.demoPQuaternion{obj.NCartesianDemo} = demo;
        end
    end
    
    methods (Access = public)
        % Figure
        function [] = plotCarteDemoXYZ(obj,dt)
            %plotCarteDemoXYZ Plot the Cartesian demo with subplot
            %   dt: scalar, the sampling period (default:0.001)
            if nargin < 2
                dt = 0.001;
            end
            figure;
            M = obj.NCartesianDemo;
            ylabels = {'x','y','z'};
            for i = 1:3
                for j = 1:M
                    subplot(3,1,i);
                    demoCarte = obj.demoCartesian{j};
                    t = (1:size(demoCarte,3))*dt - dt;
                    plot(t,permute(demoCarte(i,4,:),[3,1,2]));
                    hold on;
                end
                grid on;
                ylabel(ylabels{i});
            end
        end
        function [] = plotCarteDemoQuat(obj,dt)
            %plotCarteDemoQuat plot the unit quaternion
            figure;
            labels = {'w','x','y','z'};
            for i = 1:4
                for j = 1:obj.NCartesianDemo
                    subplot(4,1,i);
                    t = (1:size(obj.demoPQuaternion{j},1))*dt-dt;
                    plot(t,obj.demoPQuaternion{j}(:,3+i));
                    hold on;
                end
                grid on;
                ylabel(labels{i});
            end
        end
    end
    
    methods (Access = public)
        % Auxiliary functions
        function [obj, demoPQuat] = computeDemoQuaternion(obj)
            %computeDemoQuaternion Compute the unit quaternion of the demos
            %   Robotics systems toolbox is required
            %   demoQuat: 1 x M cell, [x y z w qx qy qz]
            demoPQuat = cell(size(obj.demoCartesian));
            for i = 1:obj.NCartesianDemo
                demoQuat = tform2quat(obj.demoCartesian{i});
                demoP = permute( obj.demoCartesian{i}(1:3,4,:), [3,1,2] );
                demoPQuat{i} = [demoP,demoQuat];
            end
            obj.demoPQuaternion = demoPQuat;
        end
        function [obj, demoSE3] = computeDemoCartesian(obj)
            %computeDemoCartesian Compute the SE3 demo given PQ demo
            %   Robotics systems toolbox is required
            %   demoSE3: 1 x NDemo cell, SE3 demo
            demoSE3 = cell(size(obj.demoPQuaternion));
            for i = 1:obj.NCartesianDemo
                N = size(obj.demoPQuaternion{i},1);
                demoP = obj.demoPQuaternion{i}(:,1:3);
                demoQuat = obj.demoPQuaternion{i}(:,4:7);
                demoSO3 = quat2rotm(demoQuat);
                demoSE3{i} = repmat(eye(4),[1,1,N]);
                for j = 1:N
                    demoSE3{i}(1:3,1:3,j) = demoSO3(:,:,j);
                    demoSE3{i}(1:3,4,j) = demoP(j,:)';
                end
            end
            obj.demoCartesian = demoSE3;
        end
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

