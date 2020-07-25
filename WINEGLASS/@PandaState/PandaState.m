classdef PandaState
    %PandaState Store the data of RobotState and Model
    %   Used for 
    %
    %   Haopeng Hu
    %   2020.06.29
    %   All rights reserved
    %
    %   Notations:
    %   |   O:  Base frame
    %   |   F:   Flange frame
    %   |   K:   Stiffness frame
    %   |   EE: End effector frame
    %   |   NE: Norminal end effector frame
    
    properties (Access = public)
        JP;                 % 1 x 7, Joint position
        OTEE;            % 1 x 16, End effector frame pose in base frame
        FTEE;             % 1 x 16, End effector frame pose in flange frame
        FTNE;            % 1 x 16, Norminal end effector frame pose in flange frame
        NETEE;          % 1 x 16, End effector frame pose in norminal end effector frame
        EETK;            % 1 x 16, Stiffness frame pose in end effector frame
        OFK;             % 1 x 6, Estimated externel wrench on stiffness frame in base frame
        KFK;              % 1 x 6, Estimated externel wrench on stiffness frame in stiffness frame
        TAU;             % 1 x 7, Joint torque
    end
    
    methods
        function obj = PandaState()
            %PandaState Init. a vaccum state
            %   It is not necessary to init. all the properties at once.
            
            % Normal state
            obj.JP = [];
            obj.OTEE = []; obj.FTEE = []; obj.NETEE = [];
            obj.FTNE = [];
            obj.EETK = [];
            obj.OFK = []; obj.KFK = [];
            obj.TAU = [];
        end
        
    end
end

