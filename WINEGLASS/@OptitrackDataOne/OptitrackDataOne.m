classdef OptitrackDataOne < OptitrackData
    %OptitrackDataOne Data from Optitrack MoCap system
    %   Use it for data storation and pre-process.
    %   Raw data format: frame, time, qx, qy, qz, qw, x, y, z, ..., x, y, z
    %
    %   Haopeng Hu
    %   2020.12.06
    %   All rights reserved
    %
    %   Notations:
    %   |   N:	num. of data
    %   |   Nc:	num. of columns in raw data
    %   |   Nb:	num. of rigid bodies
    %   |   Nm:	num. of markers
    %   |   Nl:	num. of lost data points
    %
    %   Recommended usage:
    %   |   obj = OptitrackDataOne(Nb, Nm);
    %   |   obj = obj.CSVRead('PATH');
    
    properties
        LossRate;       % 1 x Nb+Nm, point loss rate
        LossID;           % 1 x Nb+Nm cell of Nl x 1 array, the IDs of lost points
    end
    
    methods
        function obj = OptitrackDataOne(Nb,Nm)
            %OptitrackDataOne Initialization with Nb and Nm.
            %   Nb: integer, num. of rigid bodies
            %   Nm: integer, num. of markers
            obj = obj@OptitrackData(Nb,Nm);
        end
        
        function obj = CSVRead(obj,PATH)
            %CSVRead Read the raw .csv data. Current data will be replaced.
            %   PATH: String, the file path without '.csv'
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
                    counter = counter + 7;
                end
            end
            if obj.Nm > 0
                % Marker data
                for i = 1:obj.Nm
                    obj.marker{i} = Data(:,counter : counter+2);
                    counter = counter + 3;
                end
            end
            % Point loss rate
            [obj.LossRate, obj.LossID] = obj.getPointLossRate();
        end
        
        function [PLRate, PLIDs] = getPointLossRate(obj)
            %getPointLossRate Get the point loss rate and the lost IDs
            %   -----------------------------------------
            %   PLRate: 1 x (Nb+Nm), point loss rate of each object
            %   PLIDs: 1 x (Nb+NM) cell of Nl x 1 array, IDs of lost points
            if obj.Nb + obj.Nm > 0
                PLRate = zeros(1,obj.Nb + obj.Nm);
                PLIDs = cell(1,obj.Nb + obj.Nm);
                if obj.Nb > 0
                    for i = 1:obj.Nb
                        tmpData = obj.body{i};
                        PLRate(i) = sum(isnan(tmpData(:,1)))/size(tmpData,1);
                        tmpIDs = (1:size(tmpData,1));
                        PLIDs{i} = tmpIDs(isnan(tmpData(:,1)));
                    end
                end
                if obj.Nm > 0
                    for i = 1:obj.Nm
                        tmpData = obj.marker{i};
                        PLRate(obj.Nb+i) = sum(isnan(tmpData(:,1)))/size(tmpData,1);
                        tmpIDs = (1:size(tmpData,1));
                        PLIDs{obj.Nb+i} = tmpIDs(isnan(tmpData(:,1)));
                    end
                end
            else
                PLRate = 0;
                PLIDs = [];
            end
        end
    end
    
    methods (Access = public)
        [Data] = dataRegulate(obj,rawData);
    end
    
    methods (Hidden = true)
        [obj] = readOptitrackData(obj,dataPath,M);
        [obj] = quatWXYZ(obj);
        [obj, r] = regulateRawData(obj);
        [DataOut] = interpTraj(obj,DataIn,mode);
    end
end

