%Blueberry_import

%% Import OTEE data
%{
pathName = 'DECANTER\Blueberry\Data\10-30\';
M = 9;

% Data storation
frankaData = [];
frankaData.OTEE = [];
frankaData = repmat(frankaData,[1,M]);

for i = 1:M
    frankaData(i).OTEE = readmatrix(strcat(pathName,'lead0',int2str(i),'_OTEE.csv'));
end

figure;
for i = 1:M
    tmpData = frankaData(i).OTEE;
    plot3(tmpData(:,13),tmpData(:,14),tmpData(:,15));
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);
%}

%% Import OptitrackData data

M = 21;
optiData = repmat(OptitrackData(2,8),[1,M]);
path = 'DECANTER\Blueberry\Data\11-05\';

for i = 1:M
    optiData(i) = optiData(i).readOptitrackData(path,i);
%     optiData(i).plotMarkersXYZ();
    optiData(i).plot3Markers();
end

r = zeros(M,2+8);
for i = 1:M
    r(i,:) = optiData(i).getPointLossRate();
end
%}
