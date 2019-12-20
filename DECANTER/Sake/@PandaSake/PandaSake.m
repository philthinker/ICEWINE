classdef PandaSake < PandaZero
    %PandaSake 
    %   For L assembly with franka panda.
    %   Peter Corke's Robotics Toolbox is required.
    
    %   Haopeng Hu
    %   2019.12.08
    %   All rights reserved
    
    properties
        demoFK; % 1 x N struct array. demoFK.joint:7 x N joints; demoFK.pose: 4 x 4 x N SE3 pose.
    end
    
    methods
        function obj = PandaSake()
            %PandaSake 
            %   Init. PandaSake objects without any arguments
            obj = obj@PandaZero(true);
            demoFK.joint = zeros(7,1);
            demoFK.pose = zeros(4,4,1);
            obj.demoFK = demoFK;
        end
        
        function obj = initDemoFK(obj)
            %initDemoFK Init. the demoFK property that is a struct array
            %consists of joint and pose
            %   Never forget to add joint demo in advance
            if obj.NJointDemo >= 1
                obj.demoFK = repmat(obj.demoFK,[1,obj.NJointDemo]);
                for i = 1:obj.NJointDemo
                    % Forward kinematics
                    obj.demoFK(i).joint = obj.demoJoint{i}';
                    obj.demoFK(i).pose = obj.fkine(obj.demoJoint{i});
                end
            end
        end
        
        function Data = get_demoFK_position(obj,dt)
            %get_demoFK_position Get the positions of obj.demoFK
            %   dt: scalar, assign it once you need a time series (e.g. GMR
            %   applications)
            tmpN = 0;
            M = length(obj.demoFK);
            Ns = zeros(1,M);
            for i = 1:M
                Ns(i) = size(obj.demoFK(i).pose,3);
                tmpN = tmpN + Ns(i);
            end
            if nargin < 2
                % No dt specified
                Data = zeros(3,tmpN);
                tmpIndex = 1;
                for i = 1:M
                    Data(:,tmpIndex:tmpIndex+Ns(i)-1) = permute(obj.demoFK(i).pose(1:3,4,:),[1,3,2]);
                    tmpIndex = tmpIndex + Ns(i);
                end
            else
                % dt is specified
                Data = zeros(4,tmpN);
                tmpIndex = 1;
                for i = 1:M
                    Data(2:4,tmpIndex:tmpIndex+Ns(i)-1) = permute(obj.demoFK(i).pose(1:3,4,:),[1,3,2]);
                    Data(1,tmpIndex:tmpIndex+Ns(i)-1) = (0:Ns(i)-1)*dt;
                    tmpIndex = tmpIndex + Ns(i);
                end
            end
        end
        
        function Data = get_demoFK_joint(obj,dt)
            %get_demoFK_joint Get the joints of obj.demoFK
            %   dt: scalar, assign it once you need a time series (e.g. GMR
            %   applications)
            tmpN = 0;
            M = length(obj.demoFK);
            Ns = zeros(1,M);
            for i = 1:M
                Ns(i) = size(obj.demoFK(i).joint,2);
                tmpN = tmpN + Ns(i);
            end
            if nargin < 2
                % No dt specified
                Data = zeros(7,tmpN);
                tmpIndex = 1;
                for i = 1:M
                    Data(:,tmpIndex:tmpIndex+Ns(i)-1) = obj.demoFK(i).joint;
                    tmpIndex = tmpIndex + Ns(i);
                end
            else
                % dt is specified
                Data = zeros(8,tmpN);
                tmpIndex = 1;
                for i = 1:M
                    Data(2:end,tmpIndex:tmpIndex+Ns(i)-1) = obj.demoFK(i).joint;
                    Data(1,tmpIndex:tmpIndex+Ns(i)-1) = (0:Ns(i)-1)*dt;
                    tmpIndex = tmpIndex + Ns(i);
                end
            end
        end
        
        function obj = set_exeCartesian(obj,trajCarte)
            %set_exeCartesian Set obj.exeCartesian as trajCarte
            %   trajCarte: 3 x N or 4 x 4 x N, robot trajectory in Cartesian space.
            if size(trajCarte,1) == 3
                % Only positions in trajCarte, we assume its orientation is
                % the same as the world frame
                N = size(trajCarte,2);
                exeCartesian = eye(4);
                exeCartesian = repmat(exeCartesian,[1,1,N]);
                exeCartesian(1:3,4,:) = permute(trajCarte,[1,3,2]);
                obj.exeCartesian = exeCartesian;
            elseif size(trajCarte,1) == 4 && size(trajCarte,2) == 4
                % trajCarte is a SE3 trajectory
                obj.exeCartesian = trajCarte;
            end
        end
        
        function obj = set_exeJoint(obj,trajJoint)
            %set_exeJoint Set obj.exeJoint as trajCarte
            %   trajJoint: 7 x N or N x 7, trajectory in joint space
            if size(trajJoint,2) == 7
                if size(trajJoint,1) == 7
                    % We assume that trajJoint is 7 x N
                    obj.exeJoint = trajJoint;
                else
                    obj.exeJoint = trajJoint';
                end
            elseif size(trajJoint,1) == 7
                obj.exeJoint = trajJoint;
            end
        end
        
        function [Demo,obj] = sparse_demoFK(obj,mode)
            %sparse_demoFK_joint Sparse the data in obj.demoFK
            %   mode: integer, the sparse mode (default: 0)
            if nargin < 2
                mode = 0;
            end
            if mode == 0
                % Sparese w.r.t. by down sampling
                W = 2;  % Only keep the first element each W elements
                for i = 1:length(obj.demoFK)
                    obj.demoFK(i).joint = obj.dataThrow(obj.demoFK(i).joint, W);
                    obj.demoFK(i).pose = obj.dataThrow(obj.demoFK(i).pose, W);
                end
            end
            Demo = obj.demoFK;
        end
    end
    
    methods (Access = public)
        % Figure
        function [] = plotJoint(obj,trajs,dt)
            %plotJoint Plot the joints specified by traj
            %   traj: 1 x M cell, the joint trajectories (optional)
            %   We assume that each cell in trajs is N x 7
            if nargin >= 2
                % Once trajs is not assigned, plot joint of obj.demoFK.
                M = length(trajs);
                traj.joint = trajs{1};
                traj = repmat(traj,[M,1]);
                for i = 1:M
                    traj(i).joint = trajs{i}';
                end
                if nargin >= 3
                    % dt is assigned
                    figure;
                    for i = 1:M
                        obj.plotPandaJoint(traj(i).joint,0,dt);
                    end
                else
                    % dt is not assigned
                    figure;
                    for i = 1:M
                        obj.plotPandaJoint(traj(i).joint);
                    end
                end
            else
                % Otherwise, formulate the cell trajs into struct array
                M = obj.NJointDemo;
                traj.joint = obj.demoFK(1).joint;
                traj = repmat(traj,[M,1]);
                for i = 1:M
                    traj(i).joint = obj.demoFK(i).joint;
                end
                figure;
                for i = 1:M
                    obj.plotPandaJoint(traj(i).joint);
                end
            end
        end
        function [] = plotJointDemo(obj,exe)
            %plotJointDemo Plot the joint trajectories with exeJoint
            %   exe: boolean, true for plot obj.exeJoint together
            %   (default:false)
            if nargin < 2
                exe = false;
            end
            M = obj.NJointDemo;
            if exe
                M = M + 1;
                obj.demoJoint{M} = obj.exeJoint';
            end
            
            figure;
            for i = 1:7
                subplot(7,1,i);
                for j = 1:M
                    t = linspace(0,1,size(obj.demoJoint{j},1));
                    if exe
                        % Plot obj.exeJoint together
                        if j > obj.NJointDemo
                            plot(t,obj.demoJoint{j}(:,i),'b');
                        else
                            plot(t,obj.demoJoint{j}(:,i),'Color',[0.5,0.5,0.5]);
                        end
                    else
                        % Just plot obj.demoJoint
                        plot(t,obj.demoJoint{j}(:,i));
                    end
                    hold on;
                end
                ylabel(strcat('Joint ',int2str(i)));
                grid on;
            end
        end
        function [] = plotCarte(obj,trajs)
            %plotCarte Plot the positions
            %   trajs: 1 x M cell, the SE3 trajectories
            if nargin < 2
                % Plot the obj.demoFK.pose
                trajs = cell(1,obj.NJointDemo);
                for i = 1:obj.NJointDemo
                    trajs{i} = obj.demoFK(i).pose;
                end
            end
            M = length(trajs);
            figure;
            for i = 1:M
                tmpTraj = trajs{i};
                tmpTraj = tmpTraj(1:3,4,:);
                tmpTraj = permute(tmpTraj,[1,3,2]); % 3 x N
                plot3(tmpTraj(1,:),tmpTraj(2,:),tmpTraj(3,:));
                axis equal;
                grid on;
                hold on;
            end
        end
        function [] = plotCarteDemo(obj,exe)
            %plotCarteDemo Plot obj.demoCartesian with obj.exeCartesian
            %   exe: boolean, true for plot obj.exeCartesian together. You
            %   must set obj.exeCartesian in advance. (default: false)
            if nargin < 2
                exe = false;
            end
            M = obj.NCartesianDemo;
            if exe
                % Add the exeCartesian to the end of the Cartesian demos
                M = obj.NCartesianDemo + 1;
                obj.demoCartesian{M} = obj.exeCartesian;
            end
            LEGEND = cell(1,M); % legend
            figure;
            for i = 1:M
                LEGEND{i} = strcat('Demonstration: ',int2str(i));
                traj = permute(obj.demoCartesian{i}(1:3,4,:),[3,1,2]);   % 3 x 1 x N to N x 3
                if exe
                    % Demos are gray but obj.exeCartesian is blue
                    if i <= obj.NCartesianDemo
                        plot3(traj(:,1),traj(:,2),traj(:,3),'Color',[0.5,0.5,0.5]);
                    else
                        plot3(traj(:,1),traj(:,2),traj(:,3),'b');
                    end
                else
                    % Plot with colored lines
                    plot3(traj(:,1),traj(:,2),traj(:,3));
                end
                hold on;
            end
            if M > obj.NCartesianDemo
                % The last trajectory is exeCartesian
                LEGEND{M} = 'Generated Path';
            end
            grid on; axis equal;
            legend(LEGEND);
        end
    end
    
    methods (Access = protected)
        [] = plotPandaJoint(obj,trajJoint,T,dt);
        Data = dataThrow(obj,DataIn,W);
    end
    
end

