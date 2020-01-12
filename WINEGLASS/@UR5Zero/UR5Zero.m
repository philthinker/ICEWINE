classdef UR5Zero
    %UR5Zero Universal Robot 5kg
    %   This class is used to store the demonstration data of UR5.
    %   It can store more than one demonstration trajectories in both joint
    %   and Cartesian space (SE3)
    %
    %   Haopeng Hu
    %   2020.01.08
    %   All rights reserved
    
    properties (Access = public)
        exeJoint;       % N x 7, joint space trajectory
        exeCartesian;   % 4 x 4 x N, Cartesian space trajectory
        exeTime;        % N x 1, time series
        kModelEnable;   % boolean, is the kModel enabled?
    end
    
    properties (Access = public)
        NJointDemo;     % Num. of demos in joint space
        NCartesianDemo; % Num. of demos in Cartesian space
        demoJoint;      % 1 x NDemo cell, Demos in joint space
        demoCartesian;  % 1 x NDemo cell, Demos in Cartesian space
    end
    
    properties (Access = protected)
        InterFreq = 1000;   % 1000 Hz
        kModel;             % Kinematic model (P. Corke's Robotics Toolbox is required)
    end
    
    methods
        function obj = UR5Zero(kModelEnable)
            %UR5Zero Init. the UR5.
            %   kModelEnable: boolean, true for the property kModel needed.
            %   Maker sure P. Corke's Robotics toolbox is required if
            %   kModelEnable is set true
            obj.NJointDemo = 0;
            obj.NCartesianDemo = 0;
            obj.demoJoint = cell([1,1]);
            obj.demoCartesian = cell([1,1]);
            obj.exeJoint = zeros(1,6);
            obj.exeCartesian = eye(4,4);
            obj.exeTime = 0;
            if nargin > 0
                obj.kModelEnable = kModelEnable;
                if kModelEnable
                    mdl_ur5;
                    obj.kModel = ur5;
                    clear qr qz ur5
                end
            else
                obj.kModelEnable = false;
                obj.kModel = NaN;
            end
        end
        
        function obj = addJointDemo(obj,demo)
            %addJointDemo Add demo in joint space
            %   demo: N x 6, joint demo
            legal = true;   % Reserved for future use
            if legal
                obj.NJointDemo = obj.NJointDemo + 1;
                obj.demoJoint{obj.NJointDemo} = demo;
            end
        end
        
        function obj = addCartesianDemo(obj,demo,legal)
            %addCartesianDemo Add demo in Cartesian space
            %   demo: N x 16, Cartesian demo
            %   legal: boolean, true if demos are in SE3 form while false
            %   if demos are in 1 x 16 form. (Default: false)
            if nargin < 3
                legal = false;
            end
            if legal
                obj.NCartesianDemo = obj.NCartesianDemo + 1;
                obj.demoCartesian{obj.NCartesianDemo} = demo;
            else
                obj.NCartesianDemo = obj.NCartesianDemo + 1;
                obj.demoCartesian{obj.NCartesianDemo} = obj.demo2SE3(demo);
            end
        end
    end
    
    methods (Access = public)
        % Figures
        function [] = plotJoint(obj,id)
            %plotJoint Plot the joints
            %id: integer, the index of joint demo to be plotted
            if id == 0
                % Plot the exeJoint
                obj.plotUR5Joint(obj.exeJoint);
            else
                % Plot the id-th demo
                obj.plotUR5Joint(obj.demoJoint{id});
            end
        end
        
        function [] = plotJointDemo(obj,dt)
            %plotJointDemo Plot the demos in joint space
            %   dt: scalar, the time difference (optional). Note that if
            %   you assign this argument, the absolute time series will be
            %   applied for the figure.
            M = obj.NJointDemo;
            figure;
            for i = 1:6
                subplot(6,1,i);
                for j = 1:M
                    if nargin > 1
                        % Absolute time series
                        t = dt*(0:size(obj.demoJoint{j},1)-1);
                    else
                        % Scaled time series
                        t = linspace(0,1,size(obj.demoJoint{j},1));
                    end
                    plot(t,obj.demoJoint{j}(:,i));
                    hold on;
                    ylabel(strcat('Joint ',int2str(i)));
                end
                grid on;
            end
        end
        
        function [] = plotJointDemoPlus(obj,dt,exeJointPlus)
            %plotJointDemoPlus Plot the exeJointPlus together with demos
            %   dt: the time step
            %   exeJointPlus: N x 7, the joint traj. to be plotted, note
            %   that its very left column is the time series
            demoJointPlus = obj.demoJointChron(dt); % There is no time stamps in demoJoint
            figure;
            for i = 1:6
                subplot(6,1,i);
                for j = 1:obj.NJointDemo
                    plot(demoJointPlus{j}(:,1),demoJointPlus{j}(:,i+1),'Color',[0.36,0.36,0.36]);
                    hold on;
                end
                plot(exeJointPlus(:,1),exeJointPlus(:,i+1),'Color',[0.63,0.13,0.94]);  % It's purple
                grid on; ylabel(strcat('Joint ',int2str(i))); 
                axis([exeJointPlus(1,1),exeJointPlus(end,1),-inf,inf]);
            end
        end
        
        function [] = plotCarte(obj,id)
            %plotCarte Plot the positions
            %id: integer, the index of Cartesian demo to be plotted
            if id == 0
                % Plot the exeCartesian
                obj.plot3UR5Cartesian(obj.exeCartesian);
            else
                obj.plot3UR5Cartesian(obj.demoCartesian{id});
            end
        end
        
        function [] = plotCarteDemo(obj,exe)
            %plotCarteDemo Plot the demos in Cartesian space
            %   exe: boolean, true for plot the exeCartesian along with demos
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
                    if i == M
                        COLOR = [0,160/255,233/255];    % It's blue.
                    else
                        COLOR = [0.3, 0.3, 0.3];        % It's grey.
                    end
                    plot3(traj(:,1),traj(:,2),traj(:,3),'Color',COLOR);
                else
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
        
        % Data read
        function [demoJointPlus] = demoJointChron(obj,dt)
            %demoJointChron Add a time series to the very left of demoJoint
            %   dt: scalar, the time step
            %   demoJointPlus: 1 x NJointDemo, the demos in joint space
            %   with N x 7 joint positions whose very left column is the
            %   time series.
            demoJointPlus = cell(1,obj.NJointDemo);
            for i = 1:obj.NJointDemo
                N = size(obj.demoJoint{i},1);
                demoJointPlus{i} = [(1:N)'*dt,obj.demoJoint{i}];
            end
        end
        
        function [DataJ,N] = getDemoJoint(obj,dt,tStarting)
            %getDemoJoint Get a dataset contains all of the demos in joint
            %space. It can be used for statistic analysis
            %   dt: scalar, the time step (optional). If it is assigned,
            %   the very left column will be the time series
            %   tStarting: scalar, the starting point of time (default:0)
            %   DataJ: (N * M) x D, demo data in joint space. Not that D
            %   may be 6 or 7.
            %   N: integer, the total num. of data
            N = 0;
            for i = 1:obj.NJointDemo
                N = N + size(obj.demoJoint{i},1);
            end
            if nargin > 1
                % dt is assigned. Add the time series to the very left
                DataJ = zeros(N,7);
                if nargin < 3
                    tStarting = 0;
                end
                tmpIndex = 1;
                for i = 1:obj.NJointDemo
                    tmpN = size(obj.demoJoint{i},1);
                    DataJ(tmpIndex:tmpIndex+tmpN-1,1) = dt*(0:tmpN-1)' + tStarting;
                    DataJ(tmpIndex:tmpIndex+tmpN-1,2:end) = obj.demoJoint{i};
                    tmpIndex = tmpIndex + tmpN;
                end
            else
                % dt is not assigned.
                DataJ = zeros(N,6);
                tmpIndex = 1;
                for i = 1:obj.NJointDemo
                    tmpN = size(obj.demoJoint{i},1);
                    DataJ(tmpIndex:tmpIndex+tmpN-1,:) = obj.demoJoint{i};
                    tmpIndex = tmpIndex + tmpN;
                end
            end
        end
        
        function [obj,demoCarteFK] = demoFKReplace(obj)
            %demoFKReplace Replace the property demoCartesian with the SE3
            %data computed by UR5's forward kinematics. It works only when
            %obj.kModelEnable == true.
            %   Be careful! The original demos in Cartesian space will be
            %   discarded once you run the function!
            obj.NCartesianDemo = obj.NJointDemo;
            demoCarteFK = cell(1,obj.NJointDemo);
            if obj.kModelEnable
                for i = 1:obj.NCartesianDemo
                    demoCarteFK{i} = obj.fkine(obj.demoJoint{i});
                end
            end
            obj.demoCartesian = demoCarteFK;
        end
        
        % Trajectory Generation
        function exeJoint = jtraj(obj,routes,dT)
            %jtraj Trajectory generation by Peter Corke's jtraj function
            %   routes: K x 7, the route points with time series in the
            %   first column
            %   dT: scalar, seconds per radian
            K = size(routes,1);
            tmpJTrajSeg = cell(1,K-1);  tmpdt = zeros(K-1,1);
            for i = 2:K
                tmpdt(i-1) = ceil(max(abs(routes(i,:)-routes(i-1,:))) * dT * obj.InterFreq);
                [tmpJTrajSeg{i-1},~,~] = jtraj(routes(i-1,:),routes(i,:),tmpdt(i-1));
            end
            exeJoint = zeros(sum(tmpdt),6);
            tmpIndex = 1;
            for i = 1:K-1
                exeJoint(tmpIndex:tmpIndex+tmpdt(i)-1,:) = tmpJTrajSeg{i};
                tmpIndex = tmpIndex + tmpdt(i);
            end
        end
        
        function trajMoveit = toMoveitForm(obj,trajCarte)
            %toMoveitForm Transform the SE3 array into its moveit form:
            %[x,y,z,i,j,k,w]
            %   trajCarte: 4 x 4 x N, trajectory in SE3 form (optional). If
            %   it is not assigned, it will be set as obj.exeCartesian.
            %   trajMoveit: 7 x N, trajectory in moveit form.
            if nargin < 2
                trajCarte = obj.exeCartesian;
            end
            N = size(trajCarte,3);
            trajMoveit = zeros(N,7);
            for i = 1:N
                trajMoveit(i,1:3) = trajCarte(1:3,4,i)';
                tmpSO3 = trajCarte(1:3,1:3,i);
                % Note that the quat in MATLAB robotics system toolbox is
                % [w, x, y, z]. But that in moveit is [x, y, z, w]. It can
                % be modified. Be careful!
                tmpQuat = rotm2quat(tmpSO3);
                tmpW = tmpQuat(:,1);
                tmpQuat(:,1:3) = tmpQuat(:,2:4);
                tmpQuat(:,4) = tmpW;
                
                trajMoveit(i,4:7) = tmpQuat;
            end
        end
        
        % Kinematics
        function T = fkine(obj,joints)
            %fkine Forward kinematics
            %   joints: N x 6, the joint positions
            %   T: 4 x 4 x N, the homogeneous transformations
            N = size(joints,1);
            T = repmat(eye(4),[1,1,N]);
            if obj.kModelEnable
                for i = 1:N
                    T(:,:,i) = obj.kModel.fkine(joints(i,(1:6)));
                end
            end
        end
    end
    
    methods (Access = protected)
        demoCartesian = demo2SE3(obj,demo);
        [] = plotUR5Joint(obj,trajJoint, T);
        [] = plot3UR5Cartesian(obj,trajCartesian);
    end
    
end

