%DataImportAndPlot

%% Data import
%
M = 6;
optiData = repmat(OptitrackData(1,4),[1,M]);
path = 'DECANTER\WATER\11-29\';

for i = 1:M
    optiData(i) = optiData(i).readOptitrackData(path,i);
%     optiData(i) = optiData(i).regulateRawData();
%     optiData(i).plotMarkersXYZ();
%     optiData(i).plotBodiesXYZ();
    optiData(i).plot3Bodies();
%     optiData(i).plot3Markers();
end


%}

%% Data plot
%{
M = 1;
optiData = optiData02;
for i = 1:M
%     optiData(i).plot3Markers();
    optiData(i).plot3Bodies();
end
%}

%% Data for segmentation

Data = cell(1,M);
for i = 1:M
    Data{i} = optiData(i).getBodyData(1);
end