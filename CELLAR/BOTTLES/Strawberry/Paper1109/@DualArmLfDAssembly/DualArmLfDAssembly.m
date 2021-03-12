classdef DualArmLfDAssembly
    %DualArmLfDAssembly LfD and Dual-arm assembly
    %   We always assume the bottom part is in your left hand.
    %   Good luck!
    %
    %   Haopeng Hu
    %   2020.10.27
    %   All rights reserved
    %
    %   Notations:
    %   |   N:  num. of data
    %   |   Nb: num. of blocks/parts (2: left, right or bottom, upper)
    %   |   Nc: num. of columns
    %   |   M:  num. of demonstrations
    
    properties
        tformOMCS2Tool;     % 4 x 4 x Nb
        Nb;     % integer
        Nm;     % integer
    end
    
    methods
        % Main
        function obj = DualArmLfDAssembly()
            %DualArmLfDAssembly Good luck!
            %   No argument in
            obj.tformOMCS2Tool = repmat(eye(4),[1,1,2]);
            obj.Nb = 2;
            obj.Nm = 8;
        end
        
        function [Data] = genPartsTraj(obj,optiData,mode)
            %genPartsTraj Generate the motion traj. of the parts
            %   optiData: 1 x M OptitrackData array, raw data
            %   mode: integer, the pre-process mode (default: 0)
            %   |   0:  default mode, no pre-process
            %   |   1:  get rid of the gaps
            %   |   2:  tool center trajectory, no gap
            %   -----------------------------------------
            %   Data: 1 x M struct array
            %   |   dataL: 4 x 4 x N, data of the left part (bottom part)
            %   |   dataR: 4 x 4 x N, data of the right part (upper part)
            if nargin < 3
                mode = 0;
            end
            M = size(optiData,2);
            Data = []; Data.dataL = []; Data.dataR = [];
            Data = repmat(Data,[1,M]);
            if mode == 1 || mode == 2
                % Get rid of the gaps
            else
                % No pre-process
                tmpDataL = obj.genTrajByMarker(optiData,[1,2,3,4]);
                tmpDataR = obj.genTrajByMarker(optiData,[5,6,7,8]);
                for i = 1:M
                    Data(i).dataL = tmpDataL{i};
                    Data(i).dataR = tmpDataR{i};
                end
            end
        end
    end
    
    methods (Access = public)
        % OMCS
        function [Data] = genTrajByMarker(obj,optiData, indices)
            %genTrajByMarker Generate the body's trajectory by its related
            %markers' position data
            %   optiData: 1 x M OptitrackData array
            %   indices: 1 x 4, indices of the markers
            %   -----------------------------------------
            %   Data: 1 x M cell, SE(3) data
            M = size(optiData,2);
            Data = cell(1,M);
            for i = 1:M
                markerData = optiData(i).marker(1,indices);
                marker1 = markerData{1};
                marker2 = markerData{2};
                marker3 = markerData{3};
                marker4 = markerData{4};
                N = size(marker2,1);
                tmpData = repmat(eye(4),[1,1,N]);
                for n = 1:N
                    [euclO,euclAxis] = obj.OMCS_HRI_0929(marker1(n,:),marker2(n,:),marker3(n,:),marker4(n,:));
                    tmpData(:,:,n) = SO3P2SE3(euclAxis,euclO);
                end
                Data{i} = tmpData;
            end
        end
        
    end
    
    methods (Access = protected)
        [euclO, euclAxis] = OMCS_HRI_0929(obj, marker1, marker2, marker3, marker4);
    end
end

