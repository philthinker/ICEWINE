%Paper1127_Main
%   MoCap data of Optitrack two bodies for assembly tasks
%
%   Haopeng Hu
%   2020.12.06
%   All rights reserved
%
%
%   Raw data: 'DECANTER\WATER\'
%   Output data: 'DECANTER\WATER\'

%% Data import or load
%   Use it for new data or commit it.
%   We always assume that there are two bodies with four markers in each
%   body. Use @OptitrackDataOne for data storation.

optiData = OptitrackDataOne(2,8);
dataPath = 'DECANTER\WATER\11-10\';
M = 9;

optiData = repmat(optiData,[1,M]);
for i = 1:M
    optiData(i) = optiData(i).CSVRead(strcat(dataPath,int2str(i)));
end

% Raw data show




