classdef OptitrackData
    %OptitrackData A class used for data storation
    %   Designed especially for Optitrack Prime X41
    %   
    %   Haopeng Hu
    %   2020.09.28
    %   All rights reserved
    %
    %   Notations:
    %   |   N:      num. of data
    %   |   Nc:    num. of columns in raw data
    %   |   Nb:    num. of rigid bodies
    %   |   Nm:   num. of markers
    
    properties
        N;                % integer, num. of data
        Nb;              % integer, num. of rigid bodies
        Nm;             % integer, num. of markers
        time;            % N x 1, time series
        body;           % Nb x 1 cell, data of rigid bodies
        marker;        % Nm x 1 cell, data of markers
        estiBody;      % Nb x 1 cell, estimated rigid bodies
    end
    
    methods
        function obj = OptitrackData(Data,Nb,Nm)
            %OptitrackData Initialize the data storation
            %   Data: N x Nc, raw data
            %   Nb: integer, num. of rigid bodies
            %   Nm: integer, num. of markers
            if nargin < 1
                % Just for memory allocation
                Data = [0,0]; Nb = 0; Nm = 0;
            end
            obj.N = size(Data,1);
            obj.Nb = round(Nb);
            obj.Nm = round(Nm);
            obj.time = Data(:,2);
            % Install raw rigid body data
            tmpIndex = 3;
            if obj.Nb > 0
                obj.body = cell(1,Nb);
                for i = 1:obj.Nb
                    obj.body{i} = Data(:,tmpIndex:tmpIndex+7-1);    % qx qy qz qw x y z
                    tmpIndex = tmpIndex + 7;
                end
                obj.estiBody = obj.body;
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
        
        function [Data] = getBodyData(obj,index)
            %getBodyData Get data of some body
            %   index: integer, the body to be retrieved (default: 1)
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
            %plot3Bodies
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
            %   PLRate: 1 x M, point loss rate of each object
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
            x = xq(index);
            y = DataIn(index,:);
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
        end
    end
    
    methods (Access = protected)
        % Build-in func.
    end
end

