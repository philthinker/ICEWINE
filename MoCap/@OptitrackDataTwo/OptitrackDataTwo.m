classdef OptitrackDataTwo < OptitrackDataOne
    %OptitrackDataTwo Data from Optitrack MoCap system v2.1
    %   Use it for data storation and pre-process.
    %   - Raw data format: frame, time, qx, qy, qz, qw, x, y, z, ..., x, y, z
    %   - We always assume that each body consists of the same num. of
    %   markers.
    %   - Using the property 'bodyMarker' rather than 'body' and 'marker' if
    %   available. It is an 1 x Nb struct array whose fields are
    %   |   body: N x 7, body data [qw qx qy qz x y z]
    %   |   marker: 1 x Nbm cell, marker data of this body [x y z]
    %
    %   Haopeng Hu
    %   2021.01.22
    %   All rights reserved
    %
    %   Notations:
    %   |   N:	num. of data
    %   |   Nc:	num. of columns in raw data
    %   |   Nb:	num. of rigid bodies
    %   |   Nm:	num. of markers
    %   |   Nl:	num. of lost data points
    %   |   Nbm: num. of markers on each body
    %
    %   Recommended usage:
    %   |   obj = OptitrackDataOne(Nb, Nm);
    %   |   obj = obj.CSVRead('PATH');
    
    properties
        bodyMarker;         % 1 x Nb struct array, body and related markers
    end
    
    methods
        function obj = OptitrackDataTwo(Nb,Nm)
            %OptitrackDataTwo Initialization with Nb and Nm.
            %   Nb: integer, num. of rigid bodies
            %   Nm: integer, num. of markers
            obj = obj@OptitrackDataOne(Nb,Nm);
            obj.bodyMarker.body = [];
            obj.bodyMarker.marker = cell(1,Nm/Nb);
            obj.bodyMarker = repmat(obj.bodyMarker, [1,Nb]);
        end
        
        function obj = CSVRead(obj,PATH)
            %CSVRead Read the raw .csv data. Current data will be replaced.
            %   PATH: String, the file path without '.csv'
            %   |   Frame, Time
            %   |   Body1: qx qy qz qw x y z  MeanError
            %   |   Body1Maker1: x y z Quality
            %   |   ......
            %   |   Body2: .......
            %   |   Body2Marker1: ......
            %   |   ......
            %   |   BodyNMarker1: x y z
            %   |   ......
            %   |   Body1Marker1: x y z
            %   |   ......
            dataPath = strcat(PATH,'.csv');
            tmpData = readmatrix(dataPath);
            tmpN = size(tmpData,1);
            j = 1;
            while isnan(tmpData(j,1)) && j < tmpN
                j = j+1;
            end
            Data = tmpData(j:end,:);
            % Data installation
            obj.time = Data(:,2);
            counter = 3;
            if obj.Nb > 0
                % Body data
                for i = 1:obj.Nb
                    obj.body{i} = obj.dataRegulate(Data(:, counter : counter+6));
                    obj.bodyMarker(i).body = obj.body{i};
                    counter = counter + 8;
                    % Body marker data
                    % We always assume that the bodies share the same
                    % number of markers.
                    for j = 1:obj.Nm/obj.Nb
                        obj.bodyMarker(i).marker{j} = Data(:, counter:counter+2);
                        counter = counter + 4;
                    end
                end
            end
            % Marker data
            for i = 1:obj.Nm
                obj.marker{i} = Data(:,counter : counter+2);
                counter = counter + 3;
            end
            % Point loss rate
            [obj.LossRate, obj.LossID] = obj.getPointLossRate();
            obj.N = length(obj.time);
        end
    end
    
    methods (Access = public)
    end
end

