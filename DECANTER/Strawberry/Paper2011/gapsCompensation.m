%gapsCompensation

% Data: paper2011

%% Raw data show
%{
ylabels = {'qw','qx','qy','qz','x','y','z'};
for i = 1:M
    figure;
    tmpData = optiData(i).getBodyData(1);
    t = optiData(i).time;
    for j = 1:7 % qw qx qy qz x y z
        subplot(7,1,j);
        plot(t, tmpData(:,j));
        hold on;
        grid on;
        ylabel(ylabels{j});
    end
end
%}
%{
figure;
ylabels = {'qw','qx','qy','qz','x','y','z'};
for j = 1:7 % qw qx qy qz x y z
    subplot(7,1,j);
    plot(segTestData.time, segTestData.pose(:,j));
    hold on;
    grid on;
    ylabel(ylabels{j});
end
%}

%% Interpolation

%% GPR (position only)

% Gap gen.
%{
gap = [2.8, 3.1];
segTestData.gap = gap;
segTestData.timeGap = segTestData.time( (segTestData.time<gap(1)) | (segTestData.time>gap(2)), :);
segTestData.dataGap = segTestData.pose( (segTestData.time<gap(1)) | (segTestData.time>gap(2)), :);

figure;
for i = 1:7
    subplot(7,1,i);
    scatter(segTestData.timeGap, segTestData.dataGap(:,i),'.');
    grid on;
end
%}
% GPR 
%{
dataGP = [segTestData.timeGap'; segTestData.dataGap(:,5:7)'];
gp = GPZero(dataGP);
sigma = 1e2; W = 1e-1;
gp = gp.setParam(sigma,W,1e-4);
gp = gp.preGPR();

query = linspace(segTestData.time(1,:), segTestData.time(end,:),200);
dataGPCompen = gp.GPR(query);
%}
% Show
%{
ylabels = {'qw','qx','qy','qz','x(m)','y(m)','z(m)'};
figure;
for i = 1:3
    subplot(3,1,i);
    scatter(segTestData.time, segTestData.pose(:,4+i),36,'g','.');
    hold on;
    scatter(segTestData.timeGap, segTestData.dataGap(:,4+i),36,'b','.');
    scatter(query, dataGPCompen.Data(i+1,:),36,'r','.');
    grid on;
    ylabel(ylabels{i+4});
%     title('sigma = 1e2; W = 1e-1');
end
xlabel('t(s)');
%}

%% GPR (Cartesian space)

% Data init.
%{
dataGPTQuatPosi = [segTestData(2).timeGap';...
    segTestData(2).dataGap(:,1:4)';...
    segTestData(2).dataGap(:,5:7)'];
qa = dataGPTQuatPosi(2:5,end);
q = UnitQuat(dataGPTQuatPosi(2:5,:),0,true);
q = q.setQa(qa);
[eta,q] = q.computeEta();
dataGPTEtaPosi = [dataGPTQuatPosi(1,:); ...
    eta;...
    dataGPTQuatPosi(6:8,:)];
%}
% GPR
%{
gp = GPZero(dataGPTEtaPosi);
sigma = 1e2; W = 1e-1;
gp = gp.setParam(sigma,W,1e-4);
gp = gp.preGPR();

query = linspace(segTestData(2).time(1,:), segTestData(2).time(end,:),200);
dataGPCompen = gp.GPR(query);

dataGPCompen.DataTQuatPosi = [dataGPCompen.Data(1,:);...
    q.expMap(dataGPCompen.Data(2:4,:));...
    dataGPCompen.Data(5:7,:)];
%}
% Show
ylabels = {'qw','qx','qy','qz','x(m)','y(m)','z(m)'};
figure;
for i = 1:7
    subplot(7,1,i);
    scatter(segTestData(2).time, segTestData(2).pose(:,i),36,'g','.');
    hold on;
    scatter(segTestData(2).timeGap, segTestData(2).dataGap(:,i),36,'b','.');
    scatter(query, dataGPCompen.DataTQuatPosi(i+1,:),36,'r','.');
    grid on;
    ylabel(ylabels{i});
%     title('sigma = 1e2; W = 1e-1');
end
xlabel('t(s)');

%}
