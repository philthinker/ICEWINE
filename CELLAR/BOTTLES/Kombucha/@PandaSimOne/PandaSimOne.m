classdef PandaSimOne
    %PandaSimOne Simulation template for Franka Panda's RT control loop
    %
    %   Haopeng Hu
    %   2020.06.20
    %   All rights reserved
    
    properties (Constant)
        % Motion Identification
        JointPositions             = 0; % Constant, JointPositions output indicator
        JointVelocities            = 1; % Constant, JointVelocities output indicator
        CartesianPose            = 2; % Constant, CartesianPose output indicator
        CartesianVelocities     = 3; % Constant, CartesianPose output indicator
    end
    
    properties (Access = protected)
        kModel;        % SerialLink obj
    end
    
    methods
        function obj = PandaSimOne(kModelEnable)
            %PandaSimOne Init. the PandaSimOne object
            %   kModelEnable: boolean, true for enable kModel (Peter Corke's
            %   Robotics Toolbox is required (default:true)
            if nargin < 1
                kModelEnable = true;
            end
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
            else
                obj.kModel = [];
            end
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

