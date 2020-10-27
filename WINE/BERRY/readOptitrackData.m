function [Data] = readOptitrackData(dataPath,M)
%readOptitrackData Read data from optitrack csv file
%   dataPath: string, path of the .csv file
%   M: integer, the num. of .csv files (default:0)
%   -------------------------------------------------
%   Data: N x Nc (M == 0) or 1 x M cell, data
%   |   frame time qx qy qz qw x y z ...

if nargin < 2
    M = 0;
else
    M = max([round(M),0]);
end

if M == 0
    % Read only one file
    % The full file name is required
    tmpData = readmatrix(dataPath);
    N = size(tmpData,1);
    j = 1;
    while isnan(tmpData(j,1)) && j < N
        j = j+1;
    end
    Data = tmpData(j:end,:);
else
    % Read M files
    % We assume the last letter is the num. of file
    Data = cell(1,M);
    for i = 1:M
        tmpData = readmatrix(strcat(dataPath,int2str(i),'.csv'));
        N = size(tmpData,1);
        j = 1;
        while isnan(tmpData(j,1)) && j < N
            j = j+1;
        end
        Data{i} = tmpData(j:end,:);
    end
end

end

