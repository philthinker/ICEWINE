%main_MoCapCompen_0223_GPR
%   MoCapCompen
%   Optitrack data (@OptitrackDataTwo) trajectory filling with GPR
%   Good Luck!
%
%   Haopeng Hu
%   2021.02.23
%   All rights reserved
%
%   Exp. with 'Data\compen_mocap0223.mat'

%% Import the raw data with .csv files
%{
M = 5;
PATH = 'DECANTER\WATER\02-23\';
Data = repmat(OptitrackDataTwo(1,4), [1,M]);
for i = 1:M
    tmpPath = strcat(PATH,int2str(i));
    Data(i) = Data(i).CSVRead(tmpPath);
    Data(i).tag = 'MoCap_120fps_mm';
end
%}

%% Raw data show

