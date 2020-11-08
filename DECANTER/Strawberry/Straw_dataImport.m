%Straw_dataImport


%% Import OptitrackData data

M = 6;  % Number
Nb = 1;
Nm = 4;
optiData = repmat(OptitrackData(Nb,Nm),[1,M]);    % Type
path = 'DECANTER\Strawberry\Data\11-05\';   % Path

for i = 1:M
    optiData(i) = optiData(i).readOptitrackData(path,i);
    optiData(i) = optiData(i).quatWXYZ();
%     optiData(i).plotMarkersXYZ();
    optiData(i).plot3Markers();
end

r = zeros(M,Nb+Nm);
for i = 1:M
    r(i,:) = optiData(i).getPointLossRate();
end