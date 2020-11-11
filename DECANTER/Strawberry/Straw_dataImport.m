%Straw_dataImport


%% Import OptitrackData data

M = 9;  % Number
Nb = 2;
Nm = 8;
optiData = repmat(OptitrackData(Nb,Nm),[1,M]);    % Type
path = 'DECANTER\WATER\11-10\';                        % Path

for i = 1:M
    optiData(i) = optiData(i).readOptitrackData(path,i);
    optiData(i) = optiData(i).quatWXYZ();
    optiData(i).tag = strcat('BB_dualarm_',int2str(i));    % Tag
%     optiData(i).plotMarkersXYZ();
    optiData(i).plot3Markers();
end

r = zeros(M,Nb+Nm);
for i = 1:M
    r(i,:) = optiData(i).getPointLossRate();
end