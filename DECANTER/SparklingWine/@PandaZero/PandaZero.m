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
    end
    
    properties (Access = public)
        NJointDemo;     % Num. of demos in joint space
        NCartesianDemo; % Num. of demos in Cartesian space
        demoJoint;      % 1 x NDemo cell, Demos in joint space
        demoCartesian;  % 1 x NDemo cell, Demos in Cartesian space
    end
    
    methods
        function obj = PandaZero()
            %PandaZero Init. the Panda with the num. of demo.
            obj.NJointDemo = 0;
            obj.NCartesianDemo = 0;
            obj.demoJoint = cell([1,1]);
            obj.demoCartesian = cell([1,1]);
            obj.exeJoint = zeros(1,7);
            obj.exeCartesian = eye(4,4);
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
        
        function obj = addCartesianDemo(obj,demo)
            %addJointDemo Add demo in Cartesian space
            %   demo: N x 16, Cartesian demo
            legal = true;   % Reserved for future use
            if legal
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
        
        function [] = plotJointDemo(obj,exe)
            %plotJointDemo Plot the demos in joint space
            %   exe: boolean, true for plot the exeJoint along with demos
            %   (reserved)
%             if nargin < 2
%                 exe = false;
%             end
            M = obj.NJointDemo;
%             if exe
%                 % Add the exeJoint to the end of the joint demos
%                 M = obj.NJointDemo + 1;
%                 obj.demoJoint{M} = obj.exeJoint;
%             end
            figure;
            for i = 1:7
                subplot(7,1,i);
                for j = 1:M
                    t = linspace(0,1,size(obj.demoJoint{j},1));
                    plot(t,obj.demoJoint{j}(:,i));
                    hold on;
                end
                grid on;
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
        
        % Data sparsification (Is the repetitive data useful?)
        function obj = demoJointSparse(obj,carte)
            %demoJointSparse Sparsify the joint demos
            %   carte: boolean, true for sparsify the Cartesian demos
            %   simultaneously based on the joint features
        end
        
        function obj = demoCarteSparse(obj,joint)
            %demoCarteSparse Sparsify the Cartesian demos
            %   joint: boolean, true for sparsigy the Joint demos
            %   simultaneously based on the Cartesian features
        end
    end
    
    methods (Access = protected)
        demoCartesian = demo2SE3(obj,demo);
        [] = plotPandaJoint(obj,trajJoint, T);
        [] = plot3PandaCartesian(obj,trajCartesian);
    end
end

