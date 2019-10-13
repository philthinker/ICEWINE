classdef PandaZero
    %PandaZero This class is used to store the demonstration data of Panda.
    %It can store more than one demonstration trajectories in both joint
    %and Cartesian space (SE3)
    %
    %   Haopeng Hu
    %   2019.09.24  Lucky to Dali
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
        function obj = PandaZero(kModelEnable)
            %PandaZero Init. the Panda with the num. of demo.
            %   kModelNeeded: boolean, true for the property kModel needed.
            %   Maker sure P. Corke's Robotics toolbox is required if
            %   kModelNeeded is set true
            obj.NJointDemo = 0;
            obj.NCartesianDemo = 0;
            obj.demoJoint = cell([1,1]);
            obj.demoCartesian = cell([1,1]);
            obj.exeJoint = zeros(1,7);
            obj.exeCartesian = eye(4,4);
            obj.exeTime = 0;
            if nargin > 0
                obj.kModelEnable = kModelEnable;
                if kModelEnable
                    % DH parameters
                    SL1=Link([0       0.333       0         0        0     ],'modified');
                    SL2=Link([0       0           0         -pi/2    0     ],'modified');
                    SL3=Link([0       0.316       0         pi/2     0     ],'modified');
                    SL4=Link([0       0           0.0825    pi/2     0     ],'modified');
                    SL5=Link([0       0.384       -0.0825   -pi/2    0     ],'modified');
                    SL6=Link([0       0           0         pi/2     0     ],'modified');
                    SL7=Link([0       0.2104          0.088     pi/2     0     ],'modified');
                    obj.kModel=SerialLink([SL1 SL2 SL3 SL4 SL5 SL6 SL7],'name','Panda');
                end
            else
                obj.kModelEnable = false;
                obj.kModel = NaN;
            end
        end
        
        function obj = addJointDemo(obj,demo)
            %addJointDemo Add demo in joint space
            %   demo: N x 7, joint demo
            legal = true;   % Reserved for future use
            if legal
                obj.NJointDemo = obj.NJointDemo + 1;
                obj.demoJoint{obj.NJointDemo} = demo;
            end
        end
        
        function obj = addCartesianDemo(obj,demo,legal)
            %addJointDemo Add demo in Cartesian space
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
                obj.plotPandaJoint(obj.exeJoint);
            else
                % Plot the id-th demo
                obj.plotPandaJoint(obj.demoJoint{id});
            end
        end
        
        function [] = plotJointDemo(obj)
            %plotJointDemo Plot the demos in joint space
            M = obj.NJointDemo;
            figure;
            for i = 1:7
                subplot(7,1,i);
                for j = 1:M
                    t = linspace(0,1,size(obj.demoJoint{j},1));
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
            %   exeJointPlus: N x 8, the joint traj. to be plotted, note
            %   that its very left column is the time series
            demoJointPlus = obj.demoJointChron(dt); % There is no time stamps in demoJoint
            figure;
            for i = 1:7
                subplot(7,1,i);
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
                obj.plot3PandaCartesian(obj.exeCartesian);
            else
                obj.plot3PandaCartesian(obj.demoCartesian{id});
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
                plot3(traj(:,1),traj(:,2),traj(:,3));
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
            %   with N x 8 joint positions whose very left column is the
            %   time series.
            %   timeSeris: N x 1, the time series used to plot a figure or
            %   regression retrivel.
            demoJointPlus = cell(1,obj.NJointDemo);
            for i = 1:obj.NJointDemo
                N = size(obj.demoJoint{i},1);
                demoJointPlus{i} = [(1:N)'*dt,obj.demoJoint{i}];
            end
        end
        
        % Trajectory Generation
        function exeJoint = jtraj(obj,routes,dT)
            %jtraj Trajectory generation by Peter Corke's jtraj function
            %   routes: K x 8, the route points with time series in the
            %   first column
            %   dT: scalar, seconds per radian
            K = size(routes,1);
            tmpJTrajSeg = cell(1,K-1);  tmpdt = zeros(K-1,1);
            for i = 2:K
                tmpdt(i-1) = ceil(max(abs(routes(i,:)-routes(i-1,:))) * dT * obj.InterFreq);
                [tmpJTrajSeg{i-1},~,~] = jtraj(routes(i-1,:),routes(i,:),tmpdt(i-1));
            end
            exeJoint = zeros(sum(tmpdt),7);
            tmpIndex = 1;
            for i = 1:K-1
                exeJoint(tmpIndex:tmpIndex+tmpdt(i)-1,:) = tmpJTrajSeg{i};
                tmpIndex = tmpIndex + tmpdt(i);
            end
        end
        
        function exeJoint = jSparse(obj,exeJointIn,THD)
            %jSparse Trajectory generation for picking up some points
            %   exeJoint: N x 7, trajectory in joint space
            %   THD: scalar, the threshold (optional)
            if nargin < 3
                THD = 0.01;
            end
            N = size(exeJointIn,1);
            tmpIndex = ones(N,1);   cutRef = exeJointIn(1,:); cutIndex = 1;
            for i = 2:N-2
                tmpVariance = max(abs(exeJointIn(i,:) - cutRef));
                if tmpVariance < THD
                    if i > cutIndex
                        tmpIndex(i) = 0;
                        cutRef = exeJointIn(i+2,:);
                        cutIndex = i+2;
                    end
                end
            end
            exeJoint = exeJointIn(tmpIndex == 1,:);
        end
        
        % Kinematics
        function T = fkine(obj,joints)
            %fkine Forward kinematics
            %   joints: N x 7, the joint positions
            %   T: 4 x 4 x N, the homogeneous transformations
            N = size(joints,1);
            T = repmat(eye(4),[1,1,N]);
            if obj.kModelEnable
                for i = 1:N
                    T(:,:,i) = obj.kModel.fkine(joints(i,(1:7)));
                end
            end
        end
    end
    
    methods (Access = protected)
        demoCartesian = demo2SE3(obj,demo);
        [] = plotPandaJoint(obj,trajJoint, T);
        [] = plot3PandaCartesian(obj,trajCartesian);
    end
end

