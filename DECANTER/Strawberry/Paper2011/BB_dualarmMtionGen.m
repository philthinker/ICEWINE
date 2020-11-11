%BB_dualarmMtionGen
%   Blue board duar-arm assembly motion generation
%
%   Designed for Z. Zhao's thesis
%   
%   Haopeng Hu
%   2020.11.10
%   All rights reserved

%% Data import

% Data: 'DECANTER\WATER\11-10'
% Data storation: 'Data\BB_dualarmMotionGen.mat'
% Use: 'Strawberry_dataImport.m'

%% Raw data show

%{
figure;
for i = 1:M
    tmpData = optiData(i).getBodyData(1);
    plot3(tmpData(:,5), tmpData(:,6), tmpData(:,7),'Color', Morandi_carnation(i),'LineWidth',1.5);
    hold on;
    tmpData = optiData(i).getBodyData(2);
    plot3(tmpData(:,5), tmpData(:,6), tmpData(:,7),'Color', Morandi_carnation(i),'LineWidth',1.5);
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

figure;
ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
for i = 1:M
    tmpq = optiData(i).getBodyData(1);
    tmpq = tmpq(:,1:4);
    tmpt = optiData(i).time;
    for j = 1:4
        subplot(4,1,j);
        plot(tmpt,tmpq(:,j),'Color',Morandi_carnation(i),'LineWidth',1.5);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('t(s)');
end

figure;
ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
for i = 1:M
    tmpq = optiData(i).getBodyData(2);
    tmpq = tmpq(:,1:4);
    tmpt = optiData(i).time;
    for j = 1:4
        subplot(4,1,j);
        plot(tmpt,tmpq(:,j),'Color',Morandi_carnation(i),'LineWidth',1.5);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('t(s)');
end
%}

%% GMM data init.

% Raw data without gaps
%{
dataShow = [];
dataShow.body1 = [];
dataShow.body2 = [];
dataShow = repmat(dataShow,[1,M]);
for i = 1:M
    % Gaps free
    dataShow(i).body1 = optiData(i).getGapFreeBodyData(1,1);
    dataShow(i).body2 = optiData(i).getGapFreeBodyData(2,1);
    % Time
    dataShow(i).t = dataShow(i).body1(:,1)';
    % No time column
    dataShow(i).body1 = dataShow(i).body1(:,2:end);
    dataShow(i).body2 = dataShow(i).body2(:,2:end);
    % SE(3)
    dataShow(i).body1tform = pq2SE3(dataShow(i).body1(:,5:7)', dataShow(i).body1(:,1:4)');
    dataShow(i).body2tform = pq2SE3(dataShow(i).body2(:,5:7)', dataShow(i).body2(:,1:4)');
    % Relatve to body1 pose frame
    dataShow(i).body1tformRel = dataShow(i).body1tform;
    tmpN = size(dataShow(i).body1tformRel,3);
    for j = 1 : tmpN
        dataShow(i).body1tformRel(:,:,j) = dataShow(i).body1tform(:,:,j)\dataShow(i).body2tform(:,:,j);
    end
    % P, Q data
    tmpData = SE3toPQ(dataShow(i).body1tformRel);  % [x y z qw qx qy qz]
    dataShow(i).body1Rel = tmpData;
    dataShow(i).body1Rel(:,1:4) = tmpData(:,4:7);
    dataShow(i).body1Rel(:,5:7) = tmpData(:,1:3);           % [ qw qx qy qz x y z]
end

% Show the transformed data
figure;
for i = 1:M
    tmpData = dataShow(i).body1Rel;
    plot3(tmpData(:,5), tmpData(:,6), tmpData(:,7),'Color', Morandi_carnation(i),'LineWidth',1.5);
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(40,20);

figure;
ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
for i = 1:M
    tmpq = dataShow(i).body1Rel;
    tmpq = tmpq(:,1:4);
    tmpt = dataShow(i).t;
    for j = 1:4
        subplot(4,1,j);
        plot(tmpt,tmpq(:,j),'Color',Morandi_carnation(i),'LineWidth',1.5);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('t(s)');
end
%}
% Init. GMM data
%{
Demos = cell(1,M);  % [phase, eta, p]
for i = 1:M
    tmpQuat = UnitQuat(dataShow(i).body1Rel(:,1:4),1);
    tmpQuat = tmpQuat.setQa();
    tmpN = size(dataShow(i).body1Rel,1);
    Demos{i} = [linspace(0,10,tmpN)', tmpQuat.computeEta()', dataShow(i).body1Rel(:,5:7)];
end

% DTW
DemosDTW = iceDTW(Demos,50);

figure;
ylabels = {'\eta_x','\eta_y','\eta_z'};
for i = 1:M
    tmpData = DemosDTW{i};
    tmpEta= tmpData(:,2:4);
    tmpt = tmpData(:,1);
    for j = 1:3
        subplot(3,1,j);
        plot(tmpt,tmpEta(:,j),'Color',Morandi_carnation(i),'LineWidth',1.5);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('t(s)');
end

figure;
ylabels = {'x', 'y', 'z'};
for i = 1:M
    tmpData = DemosDTW{i};
    tmpq = tmpData(:,5:7);
    tmpt = tmpData(:,1);
    for j = 1:3
        subplot(3,1,j);
        plot(tmpt,tmpq(:,j),'Color',Morandi_carnation(i),'LineWidth',1.5);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('t(s)');
end

for i = 1:M
    DemosDTW{i} = DemosDTW{i}';
end
%}

%% GMM and GMR

% GMM
%{
gmm = GMMOne(9,7);
gmm = gmm.initGMMTimeBased(gmm.dataRegulate(DemosDTW));
gmm = gmm.learnGMM(gmm.dataRegulate(DemosDTW));

% GMR
query = linspace(0,10,1000);
[expData, expSigma] = gmm.GMR(query);

% Compute unit quat
qa = dataShow(1).body1Rel(end,1:4)';
tmpQuat = UnitQuat(qa,0);
tmpQuat = tmpQuat.setQa();
expQuat = tmpQuat.expMap(expData(1:3,:));
%}

% Show
%{
figure;
for i = 1:M
    tmpData = dataShow(i).body1Rel;
    plot3(tmpData(:,5), tmpData(:,6), tmpData(:,7),'Color', Morandi_carnation(1),'LineWidth',1.0);
    hold on;
end
tmpData = expData(4:6,:);
plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color', Morandi_carnation(2),'LineWidth',2.0);
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(40,20);

figure;
ylabels = {'x', 'y', 'z'};
for i = 1:M
    tmpq = dataShow(i).body1Rel;
    tmpq = tmpq(:,5:7);
    tmpt = linspace(0,10,size(dataShow(i).t,2));
    for j = 1:3
        subplot(3,1,j);
        plot(tmpt,tmpq(:,j),'Color',Morandi_carnation(1),'LineWidth',1.0);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('Phase');
end
tmpData = expData(4:6,:);
tmpt = query;
for j = 1:3
    subplot(3,1,j);
    plot(tmpt,tmpData(j,:),'Color',Morandi_carnation(2),'LineWidth',2.0);
end

figure;
ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
for i = 1:M
    tmpq = dataShow(i).body1Rel;
    tmpq = tmpq(:,1:4);
    tmpt = linspace(0,10,size(dataShow(i).t,2));
    for j = 1:4
        subplot(4,1,j);
        plot(tmpt,tmpq(:,j),'Color',Morandi_carnation(1),'LineWidth',1.0);
        hold on; grid on;
        ylabel(ylabels{j});
    end
    xlabel('Phase');
end
tmpData = expQuat;
tmpt = query;
for j = 1:4
    subplot(4,1,j);
    plot(tmpt,tmpData(j,:),'Color',Morandi_carnation(2),'LineWidth',2.0);
end
%}

%% Motion assignment

% Data init.
%
exp = [];
exp.position = expData(4:6,:);
exp.quat = expQuat;
exp.axang = quat2axang(exp.quat')';
ylabels = {'x','y','z','angle'};
for i = 1:4
    subplot(4,1,i);
    plot(query, exp.axang(i,:));
    grid on;
    ylabel(ylabels{i});
end
% expTform = pq2SE3(expPQ(1:3,:),expPQ(4:7,:));
% for i = 1:size(expTform,3)
%     expTform(:,:,i) = expTform(:,:,end)\expTform(:,:,i);
% end
%}
% Motion assignment
%{
expEul = tform2eul( expTform );
expRPY = tr2rpy(expTform);
tmpEul = expEul;
tmpEul(:,2) = tmpEul(:,2)/2;    % Z Y X rpy
trajLowerEul = tmpEul;
trajLower = eul2tform(tmpEul);
trajUpper = trajLower;
for i = 1:size(trajLower,3)
    trajUpper(:,:,i) = trajLower(:,:,i)\expTform(:,:,i);
end
trajUpperEul = tform2eul(trajUpper);

% Show
figure;
tmpData = permute(trajUpper(1:3,4,:), [1,3,2]);
plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', Morandi_carnation(2), 'LineWidth',1.5);
grid on; axis equal; xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

figure;
ylabels = {'r_{z}(rad)','r_{y}(rad)','r_{x}(rad)'};
tmpData = trajUpperEul';
tmpt = query;
for i = 1:3
    subplot(3,1,i);
    plot(tmpt, tmpData(i,:), 'Color', Morandi_carnation(2),'LineWidth',1.5);
    grid on;
    ylabel(ylabels{i});
end
xlabel('Phase');

figure;
ylabels = {'r_{z}(rad)','r_{y}(rad)','r_{x}(rad)'};
tmpData = trajLowerEul';
tmpt = query;
for i = 1:3
    subplot(3,1,i);
    plot(tmpt, tmpData(i,:), 'Color', Morandi_carnation(2),'LineWidth',1.5);
    grid on;
    ylabel(ylabels{i});
end
xlabel('Phase');
%}

%% Critical points


