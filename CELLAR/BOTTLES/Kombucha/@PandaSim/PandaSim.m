classdef PandaSim
    %PandaSim Simulation template for Franka Panda's RT control loop
    %   Peter Corke's Robotics Toolbox is required
    %
    %   Haopeng Hu
    %   2020.06.17
    %   All rights reserved
    
    properties (Access = public)
        motions;       % N x D, motion memory
    end
    
    properties (Constant)
        JointPositions             = 0; % Constant, JointPositions output indicator
        JointVelocities            = 1; % Constant, JointVelocities output indicator
        CartesianPose            = 2; % Constant, CartesianPose output indicator
        CartesianVelocities     = 3; % Constant, CartesianPose output indicator
    end
    
    properties (Access = protected)
        period;         % scalar, toSec or toMSec, control period
        iCounter;      % integer, iteration counter
        kModel;        % SerialLink obj
    end
    
    methods
        function obj = PandaSim(period, motionID, NMax)
            %PandaSim Init. the simulation template
            %   period: scalar, 1(ms) or 0.001(s)
            if nargin < 3
                NMax = 100000;
            end
            if period < 1
                obj.period = 0.001;
            else
                obj.period = 1;
            end
            obj.iCounter = 0;
            if motionID == PandaSim.JointPositions || motionID == PandaSim.JointVelocities
                obj.motions = zeros(NMax,7);
            elseif motionID == PandaSim.CartesianPose
                obj.motions = zeros(NMax,16);
            elseif motionID == PandaSim.CartesianVelocities
                obj.motions = zeros(NMax, 6);
            end
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
        
        function obj = simUpdate(obj, motion)
            %simUpdate Update the simulation state
            %   motion: 1 x 7 or 1 x 16, new motion
            obj.iCounter = obj.iCounter + 1;
            obj.motions(obj.iCounter,:) = motion;
        end
        
        function [obj, iCount] = simTerminate(obj)
            %simTerminate Terminate the control loop
            obj.motions = obj.motions(1:obj.iCounter,:);
            iCount = obj.iCounter;
            obj.iCounter = 0;
        end
    end
    
    methods (Access = public)
        function counter = getCounter(obj)
            %getCounter Get the iCounter property
            counter = obj.iCounter;
        end
        
        function period = getPeriod(obj)
            %getPeriod Get the control period
            period = obj.period;
        end
        
        function period = toSec(obj)
            %toSec Return 0.001
            period = 0.001;
        end
        
        function period = toMSec(obj)
            %toMSec Return 1
            period = 1;
        end
    end
end

