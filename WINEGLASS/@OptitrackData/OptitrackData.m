classdef OptitrackData
    %OptitrackData A class used for data storation
    %   Designed especially for Optitrack Prime X41
    %   
    %   Haopeng Hu
    %   2020.09.28
    %   All rights reserved
    %
    %   Notations:
    %   |   N:  num. of data
    %   |   Nc:	num. of columns in raw data
    %   |   Nb:	num. of rigid bodies
    %   |   Nm:	num. of markers
    
    properties
        N;                % integer, num. of data
        Nb;              % integer, num. of rigid bodies
        Nm;             % integer, num. of markers
        time;            % N x 1, time series
        body;           % 1 x Nb cell, data of rigid bodies
        marker;        % 1 x Nm cell, data of markers
    end
    
    methods
        function obj = OptitrackData(Nb,Nm,Data)
            %OptitrackData Initialize the data storation
            %   Data: N x Nc, raw data
            %   |   frame, time, qx, qy, qz, x, y, z, ..., x, y, z
            %   Nb: integer, num. of rigid bodies
            %   Nm: integer, num. of markers
            if nargin < 3
                % Just for memory allocation
                %  But you must assign Nb and Nm.
                Data = [0,0,0,0];
                obj.N = size(Data,1);
                obj.Nb = round(Nb);
                obj.Nm = round(Nm);
                return;
            end
            obj.N = size(Data,1);
            obj.Nb = round(Nb);
            obj.Nm = round(Nm);
            obj.time = Data(:,2);
            % Install raw rigid body data
            tmpIndex = 3;
            if obj.Nb > 0
                % We always assume that the bodies' data comes firstly.
                obj.body = cell(1,Nb);
                for i = 1:obj.Nb
                    obj.body{i} = Data(:,tmpIndex:tmpIndex+7-1);    % qx qy qz qw x y z
                    tmpIndex = tmpIndex + 7;
                end
            end
            % Install raw marker data
            if obj.Nm > 0
                obj.marker = cell(1,Nm);
                for i = 1:obj.Nm
                    obj.marker{i} = Data(:,tmpIndex:tmpIndex+3-1);  % x y z
                    tmpIndex = tmpIndex + 3;
                end
            end
        end
        
        function [obj] = readOptitrackData(obj,dataPath,M)
            %readOptitrackData Read the data from a data file.
            %   Note that the current data will be replaced.
            %   It is used to store data into vacuum object
            %   dataPath: String, the path of the data file without '.csv'
            %   We assume the last letter is a number.
            %   M: Integer, the number of data file. Once M == 0, you must
            %   give the full name with '.csv' (default: 0)
            if nargin < 3
                M = 0;
            else
                M = max([round(M),0]);
            end
            if M == 0
                % Read only one file
                % The full file name is required
                tmpData = readmatrix(dataPath);
                tmpN = size(tmpData,1);
                j = 1;
                while isnan(tmpData(j,1)) && j < tmpN
                    j = j+1;
                end
                Data = tmpData(j:end,:);
            else
                % Read the M-th file
                % We assume the last letter is the num. of file
                tmpData = readmatrix(strcat(dataPath,int2str(M),'.csv'));
                tmpN = size(tmpData,1);
                j = 1;
                while isnan(tmpData(j,1)) && j < tmpN
                    j = j+1;
                end
                Data = tmpData(j:end,:);
            end
            obj = OptitrackData(obj.Nb,obj.Nm,Data);
        end
        
        function [Data] = getBodyData(obj,index)
            %getBodyData Get data of some body
            %   index: integer, the body to be retrieved (default: 1)
            %   -----------------------------------------
            %   Data: N x 7, the trajectory data
            if nargin < 2
                index = 1;
            end
            index = max([round(index),1]);
            if obj.Nb > 0
                Data = obj.body{index};
            else
                Data = [];
            end
        end
        
        function [Data] = getMarkerData(obj,index)
            %getMarkerData Get data of some marker
            %   index: integer, the marker to be retrieved (default: 1)
            %   -----------------------------------------
            %   Data: N x 3, the trajectory data
            if nargin < 2
                index = 1;
            end
            index = max([round(index),1]);
            if obj.Nm > 0
                Data = obj.marker{index};
            else
                Data = [];
            end
        end
        
        function [Data,gapIndices] = getGapFreeBodyData(obj,index,mode)
            %getGapFreeBodyData Get the body's trajectory without gap
            %   index: integer, the index of bodies
            %   mode: integer, 1 for data with time seq. to the left column
            %   (default: 0)
            %   -----------------------------------------
            %   Data: N x 7 or N x 8, body's trajectory
            %   gapIndices: N x 1, the logical indices of lost points
            if nargin < 3
                mode = 0;
            end
            index = min(round(index(1,1)),obj.Nb);
            tmpData = obj.body{index};
            logicalIndices = ~isnan(tmpData(:,1));
            if mode == 1
                % Time seq. to the left
                Data = [obj.time(logicalIndices), tmpData(logicalIndices,:)];
            else
                % No time seq. to the left
                Data = tmpData(logicalIndices,:);
            end
            gapIndices = ~logicalIndices;
        end
        
        function [Data,gapIndices] = getGapFreeMarkerData(obj,index,mode)
            %getGapFreeMarkerData Get the marker's trajectory without gap
            %   index: integer, the index of bodies
            %   mode: integer, 1 for data with time seq. to the left column
            %   (default: 0)
            %   -----------------------------------------
            %   Data: N x 3 or N x 4, marker's trajectory
            %   gapIndices: N x 1, the logical indices of lost points
            if nargin < 3
                mode = 0;
            end
            index = min(round(index(1,1)),obj.Nm);
            tmpData = obj.marker{index};
            logicalIndices = ~isnan(tmpData(:,1));
            if mode == 1
                % Time seq. to the left
                Data = [obj.time(logicalIndices), tmpData(logicalIndices,:)];
            else
                % No time seq. to the left
                Data = tmpData(logicalIndices,:);
            end
            gapIndices = ~logicalIndices;
        end
    end
    
    methods (Access = public)
        % Figure
        function [] = plotBodiesXYZ(obj)
            %plotBodies Plot the x y z data of bodies
            figure;
            ylabels = {'x','y','z'};
            for i = 1:obj.Nb
                tmpXYZ = obj.body{i};
                for j = 1:3
                    subplot(3,1,j);
                    plot(obj.time, tmpXYZ(:,j));
                    hold on;
                    grid on;
                    axis([0,obj.time(end),-Inf,Inf]);
                    ylabel(ylabels{j});
                    legend;
                end
            end
        end
        function [] = plot3Bodies(obj)
            %plot3Bodies Plot the bodies data (x y z)
            figure;
            for i = 1:obj.Nb
                tmpXYZ = obj.body{i};
                plot3(tmpXYZ(:,1), tmpXYZ(:,2), tmpXYZ(:,3));
                hold on;
            end
            grid on; axis equal;
            legend;
            view(3);
        end
        function [] = plotMarkersXYZ(obj)
            %plotMarkersXYZ Plot the position of markers
            if obj.Nm > 0
                figure;
                t = obj.time;
                ylabels = {'x','y','z'};
                for i = 1:obj.Nm
                    tmpXYZ = obj.marker{i};
                    for j = 1:3
                        subplot(3,1,j);
                        plot(t, tmpXYZ(:,j));
                        hold on;    grid on;
                        ylabel(ylabels{j});
                        axis([0,t(end),-Inf,Inf]);
                    end
                end
            end
        end
        function [] = plot3Markers(obj)
            %plot3Markers Plot3 the position of markers
            if obj.Nm > 0
                figure;
                for i = 1:obj.Nm
                    tmpXYZ = obj.marker{i};
                    plot3(tmpXYZ(:,1),tmpXYZ(:,2),tmpXYZ(:,3));
                    hold on; 
                end
                grid on;
                axis equal;
                view(3);
            end
        end
        % State monitor
        function [PLRate] = getPointLossRate(obj)
            %getPointLossRate Get the point loss rate
            %   PLRate: 1 x (Nb+Nm), point loss rate of each object
            if obj.Nb + obj.Nm > 0
                PLRate = zeros(1,obj.Nb + obj.Nm);
                if obj.Nb > 0
                    for i = 1:obj.Nb
                        tmpData = obj.body{i};
                        PLRate(i) = sum(isnan(tmpData(:,1)))/size(tmpData,1);
                    end
                end
                if obj.Nm > 0
                    for i = 1:obj.Nm
                        tmpData = obj.marker{i};
                        PLRate(obj.Nb+i) = sum(isnan(tmpData(:,1)))/size(tmpData,1);
                    end
                end
            else
                PLRate = 0;
            end
        end
        % Raw data pre-process
        function [DataOut] = interpTraj(obj,DataIn,mode)
            %interpChip Interpolate the data with MATLAB functions
            %   DataIn: N x D, data with NaN
            %   mode: integer, interpolation mode (default: 0)
            %   |   0: makima
            %   |   1: pchip
            %   |   2: spline
            %   -----------------------------------------
            %   DataOut: N x D, data without NaN
            if nargin < 3
                mode = 0;
            end
            % We take the first column of data as baseline
            xq = obj.time;
            index = ~isnan(DataIn(:,1)); 
            x = xq(index)';
            y = DataIn(index,:)';
            xq = xq';
            if mode == 1
                % pchip
                DataOut = pchip(x,y,xq);
            elseif mode == 2
                % spline
                DataOut = spline(x,y,xq);
            else
                % makima
                % Note that only row vector is supported
                DataOut = makima(x,y,xq);
            end
            DataOut = DataOut';
        end
        % Data pre-process
        function [obj] = markerReorder(obj,order)
            %markerReorder Reorder the marker's data with a new order
            %   order:  1 x Nm integer, new order
            if size(order,2) == obj.Nm
                order = round(order(1,:));
                tmpMarker = obj.marker;
                for i = 1:obj.Nm
                    obj.marker{i} = tmpMarker{order(i)};
                end
            else
                error('The column num. of order must equal to Nm');
            end
        end
        function [obj] = bodyReorder(obj,order)
            %bodyRecorder Reorder the body's data with a new order
            %   order:  1 x Nb integer, new order
            if size(order,2) == obj.Nb
                order = round(order(1,:));
                tmpBody = obj.body;
                for i = 1:obj.Nb
                    obj.body{i} = tmpBody{order(i)};
                end
            else
                error('The column num. of order must equal to Nb');
            end
        end
        function [obj] = quatWXYZ(obj)
            %quatWXYZ Adjust the optitrack body data 
            %[qx qy qz qw x y z] to [ qw qx qy qz x y z]
            if obj.Nb <= 0
                return;
            end
            for i = 1:obj.Nb
                tmpData = obj.body{i};
                tmpQuat  = zeros(size(tmpData,1),4);
                tmpQuat(:,1) = tmpData(:,4);
                tmpQuat(:,2:4) = tmpData(:,1:3);
                tmpData(:,1:4) = tmpQuat;
                obj.body{i} = tmpData;
            end
        end
    end
end

