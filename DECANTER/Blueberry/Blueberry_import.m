%Blueberry_import

%% Import OTEE data
%{
pathName = 'DECANTER\Blueberry\Data\10-17\';
N = 9;

% Data storation
Data = [];
Data.OTEE_app = [];
Data.OTEE_ass = [];
Data = repmat(Data,[1,N]);

for i = 1:N
    Data(i).OTEE_app = readmatrix(strcat(pathName,'lead_app_0',int2str(i),'_OTEE.csv'));
    Data(i).OTEE_ass = readmatrix(strcat(pathName,'lead_ass_0',int2str(i),'_OTEE.csv'));
end

figure;
for i = 1:N
    tmpData = Data(i).OTEE_app;
    plot3(tmpData(:,13),tmpData(:,14),tmpData(:,15));
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);
%}

%% Import OptitrackData data

M = 7;
optiData = repmat(OptitrackData(2,8),[1,M]);
path = 'DECANTER\Blueberry\Data\10-26\00';

for i = 1:M
    optiData(i) = optiData(i).readOptitrackData(path,i);
end

