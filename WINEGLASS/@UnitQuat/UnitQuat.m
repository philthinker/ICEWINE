classdef UnitQuat
    %UnitQuat Unit quaternion series
    %   For easy use of unit quaternion  in LfD studies
    %   We always claim the format: [w x y z]'
    %
    %   Haopeng Hu
    %   2020.11.08
    %   All rights reserved
    %
    %   Notations:
    %   |   N: num. of data
    
    properties
        q;      % 4 x N, quats
        qa;    % 4 x 1, auxiliary quat
        eta;   % 3 x N, eta in tangent space centered at qa
    end
    
    methods
        function obj = UnitQuat(Data,mod,flag)
            %UnitQuat Init. the object with unit quat data
            %   Data: 4 x N, N x 4, 3 x 3 x N, 4 x 4 x N, orientation data
            %   mod: integer, the data type
            %   |   0: 4 x N, q, quat data
            %   |   1: N x 4, q', quat data
            %   |   2: 3 x 3 x N, SO3 data
            %   |   3: 4 x 4 x N, SE3 data
            %   flag: boolean, true for quat. regulation (default:false)
            if nargin < 3
                flag = false;
            end
            if mod == 1
                obj.q = Data';
            elseif mod == 2
                obj.q = rotm2quat(Data)';
            elseif mod == 3
                obj.q = tform2quat(Data)';
            else
                obj.q = Data;
            end
            if flag
                % quat regulation
            end
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

