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
%{
for i = 1:M
    Data(i).plot3Bodies();
    Data(i).plotBodiesXYZ();
end
%}

%% Simple compensation via GPR (Original data are not replaced)
%{
DataPre = Data;
for i = 1:M
    Data(i).plotBodiesXYZ();
    DataPre(i) = Data(i).simpleGPRCompensation_bodyMarker(false, [1e0,1e-1,1e-2]);
    DataPre(i).plotBodiesXYZ();
end
%}

% Just load 'Data\compen_mocap0223.mat' and ignore the codes above.
% Those codes are summarized here for future use.
%{
M = 5;
PATH = 'DECANTER\WATER\02-23\';
Data = repmat(OptitrackDataTwo(1,4), [1,M]);
for i = 1:M
    tmpPath = strcat(PATH,int2str(i));
    Data(i) = Data(i).CSVRead(tmpPath);
    Data(i).tag = 'MoCap_120fps_mm';
    Data(i) = Data(i).simpleGPRCompensation_bodyMarker(false);
end
%}

% Run the following codes at once.
% Results are stored in 'Data\compen_mocap0315.mat'.
%% Build the data struct for simplicity
%{
MC = 4;

DataTest = [];
DataTest.bodyOrigin = [];
DataTest.markerOrigin = [];
DataTest.time = [];
DataTest.cutLogiIDs = [];

DataTest =repmat(DataTest,[1,M*MC]);
tmpCnt = 1;
for i = 1:M
    k = 1;
    while k <= MC
        DataTest(tmpCnt).time = DataPre(i).time;
        DataTest(tmpCnt).bodyOrigin = DataPre(i).body{1};
        % Compute eta %%%%%%%%%%%%%%%
        tmpQuat = DataTest(tmpCnt).bodyOrigin(:,1:4)';
        tmpQa = tmpQuat(:,end);
        tmpEta = quatLogMap(tmpQuat,tmpQa);
        DataTest(tmpCnt).bodyOriginEtaP = [tmpEta', DataTest(tmpCnt).bodyOrigin(:,5:7)];
        %%%%%%%%%%%%%%%%%%%%%%%%
        DataTest(tmpCnt).markerOrigin = cell(1,DataPre(i).Nm);
        for j = 1:DataPre(i).Nm
            DataTest(tmpCnt).markerOrigin{j} = DataPre(i).marker{j};
        end
        k = k + 1;
        tmpCnt = tmpCnt + 1;
    end
end
%
%% Set the cutting points
%
tmpCnt = 1;
for i = 1:length(DataTest)
    [~,DataTest(i).cutLogiIDs,~] = setCuttingPoints(DataTest(i).bodyOrigin,200,200+tmpCnt*50-1,50);
    if mod(tmpCnt,MC) == 0
        tmpCnt = 1;
    else
        tmpCnt = tmpCnt + 1;
    end
end
%
%% Compensation via GPR
for i = 1:length(DataTest)
    x = DataTest(i).time;
    y = DataTest(i).bodyOrigin;
    y(~DataTest(i).cutLogiIDs,:) = NaN;
    [~, DataTest(i).compenGPR, ~] = simpleGPRCompen(y,true,x,[1e1,1e0,1e-3]);
end
%% Compensation via spline, makima, pchip
%
for i = 1:length(DataTest)
    x = DataTest(i).time(DataTest(i).cutLogiIDs)';
    y = DataTest(i).bodyOriginEtaP(DataTest(i).cutLogiIDs,:)';
    xq = DataTest(i).time(~DataTest(i).cutLogiIDs)';
    tmpQuat = DataTest(i).bodyOrigin(:,1:4)';
    tmpQa = tmpQuat(:,end);
    tmpData = spline(x,y,xq);
    DataTest(i).compenSpline = [quatExpMap(tmpData(1:3,:),tmpQa)', tmpData(4:6,:)'];
    tmpData = pchip(x,y,xq);
    DataTest(i).compenPchip = [quatExpMap(tmpData(1:3,:),tmpQa)', tmpData(4:6,:)'];
    tmpData = makima(x,y,xq);
    DataTest(i).compenMakima = [quatExpMap(tmpData(1:3,:),tmpQa)', tmpData(4:6,:)'];
end
%}
%% Compensation via our method
%
for i = 1:length(DataTest)
end
%
%% Comparison
%
for i = 1:MC
    figure;
    x = DataTest(i).time;
    y = DataTest(i).bodyOrigin;
    xq = x(~DataTest(i).cutLogiIDs);
    yqGPR = DataTest(i).compenGPR;
    yqSpline = DataTest(i).compenSpline;
    yqPchip = DataTest(i).compenPchip;
    yqMakima = DataTest(i).compenMakima;
    for j = 1:7
        subplot(7,1,j);
        plot(x,y(:,j),'k');
        hold on;
        plot(xq,yqGPR(:,j),'b');
        plot(xq,yqSpline(:,j),'r');
        plot(xq,yqPchip(:,j),'g');
        plot(xq,yqMakima(:,j),'y');
    end
    
end
