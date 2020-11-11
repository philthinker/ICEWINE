%MCA_dataPreprocess_demo
%   Reserved for Z. Cao and Z. Zhao's thesis
%
%   Haopeng Hu
%   2020.11.09
%   All rights reserved

% Raw data: 'Data\Data1030_opti2.mat'
% Data: 'Data\MCA_dataPreprocess.mat'

%% Data storation

%{
dataShow = [];
dataShow.body1raw = [];
dataShow.body2raw = [];
dataShow.t = [];
dataShow = repmat(dataShow,[1,M]);
for i = 1:M
    dataShow(i).body1raw = optiData(i).getBodyData(1);  % qw qx qy qz w x y z
    dataShow(i).body2raw = optiData(i).getBodyData(2);
    dataShow(i).t = optiData(i).time;
end
%}

%% Raw data

% Plot3
%{
labels = {'q_w','q_x','q_y','q_z','x(m)','y(m)','z(m)'};
figure;
for i = 1:M
    tmpData = dataShow(i).body1raw;
    plot3(tmpData(:,5),tmpData(:,6),tmpData(:,7),'Color',Morandi_carnation(i),'LineWidth',1.0);
    hold on;
    tmpData = dataShow(i).body2raw;
    plot3(tmpData(:,5),tmpData(:,6),tmpData(:,7),'Color',Morandi_carnation(i),'LineWidth',1.0);
end
grid on; axis equal; xlabel(labels{5}); ylabel(labels{6}); zlabel(labels{7});
view(3);
%}

% Plot
%{
% typical: 6
for i = 7:M
    figure(i);
    t = dataShow(i).t;
    tmpData = dataShow(i).body1raw;
    for j = 1:7
        subplot(7,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_carnation(i),'LineWidth',1.5);
        grid on;
        ylabel(labels{j});
    end
    xlabel('t(s)');
end
%}

%% Gapped data

%{
i= 6;
[dataShow(i).body1rawGapped, ~] = dataGap([2.725, 3.225], dataShow(i).t, dataShow(i).body1raw);
[dataShow(i).body1rawGapped, dataShow(i).body1rawGappedID] = dataGap([5.150, 5.350], dataShow(i).t, dataShow(i).body1rawGapped);
dataShow(i).body1rawGapped(:,1:4) = quatRegulate(dataShow(i).body1rawGapped(:,1:4), true);
for i = 6:6
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawGapped;
    for j = 1:7
        subplot(7,1,j);
        plot(t,tmpData(:,j),'Color',[0.0, 0.0, 1.0],'LineWidth',1.5);
        grid on;
        ylabel(labels{j});
    end
    xlabel('t(s)');
end
%}

%% Compensation

% GPR data init.
%{
for i = 6:6
    tmpTQuatP = [...
        dataShow(i).t(~dataShow(i).body1rawGappedID)';...
        dataShow(i).body1rawGapped(~dataShow(i).body1rawGappedID,:)'];
    tmpQuat = UnitQuat(tmpTQuatP(2:5,:),0,true);
    tmpQuat = tmpQuat.setQa();
    tmpEta = tmpQuat.computeEta();
    tmpTEtaP = [...
        tmpTQuatP(1,:);...
        tmpEta;...
        tmpTQuatP(6:8,:)];
    dataShow(i).BodyTEtaP1 = tmpTEtaP;
    dataShow(i).BodyQuat1 = tmpQuat;
end
%}

% GPR
%{
for i = 6:6
    gp = GPZero(dataShow(i).BodyTEtaP1);
    sigma = 1e-1; W = 1e-1;
    gp = gp.setParam(sigma,W,1e-4);
    gp = gp.preGPR();
    
    query = linspace(dataShow(i).t(1), dataShow(i).t(end),1000);
    dataGPCompen = gp.GPR(query);
    
    dataGPCompen.DataTQuatPosi = [dataGPCompen.Data(1,:);...
        dataShow(i).BodyQuat1.expMap(dataGPCompen.Data(2:4,:));...
        dataGPCompen.Data(5:7,:)];
    dataShow(i).BodyDataGPComp1 = dataGPCompen;
end
%}

% Show
%{
for m = 6:6
    figure;
    for i = 1:7
        subplot(7,1,i);
        t = dataGPCompen.DataTQuatPosi(1,:);
        tmpData = dataGPCompen.DataTQuatPosi(2:8,:);
        scatter(t, tmpData(i,:),36,'r','.');
        hold on;
        scatter(dataShow(m).t, dataShow(m).body1rawGapped(:,i),36,'b','.');
        grid on;
        ylabel(labels{i});
        if i == 1
%             title('sigma = 1e-1; W = 1e-1;');
        end
    end
    xlabel('t(s)');
end
%}

%% Outlier detection

% Raw Data show
%{
% Typical: 12
for i = 12:12
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1raw;
    for j = 1:3
        subplot(3,1,j);
        plot(t,tmpData(:,j+4),'Color',Morandi_carnation(2),'LineWidth',1.5);
        grid on;
        ylabel(labels{j+4});
    end
    xlabel('t(s)');
end
%}

% Data noise
%{
for i = 12:12
    dataShow(i).body1rawNoisy = dataShow(i).body1raw(:,5:7);
    dataShow(i).body1rawNoisy(553,1) = -0.055;
    dataShow(i).body1rawNoisy(553,2) = 0.132;
    dataShow(i).body1rawNoisy(553,3) = -0.063;
    % Show
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawNoisy;
    for j = 1:3
        subplot(3,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_carnation(2),'LineWidth',1.5);
        grid on;
        ylabel(labels{j+4});
    end
    xlabel('t(s)');
end
%}

% Compute vel.
%{
for i = 12:12
    dataShow(i).body1rawNoisyVel = finiteDiff(dataShow(i).body1rawNoisy, 1/120);
    % Show
    figure;
    ylabels = {'v_{x}(m/s)', 'v_{y}(m/s)', 'v_{z}(m/s)'};
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawNoisyVel;
    for j = 1:3
        subplot(3,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_carnation(3),'LineWidth',1.5);
        grid on;
        ylabel(ylabels{j});
    end
    xlabel('t(s)');
end
%}

% Outlier detection
%{
for i = 12:12
    dataShow(i).body1rawNoisyOutlier = [553];
    % Show
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawNoisy;
    outlier = dataShow(i).body1rawNoisyOutlier;
    for j = 1:3
        subplot(3,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_carnation(2),'LineWidth',1.5);
        grid on; hold on;
        ylabel(labels{j+4});
        scatter(t(outlier),tmpData(outlier,j),120,Morandi_carnation(1),'LineWidth',1.5);
        axis([-Inf, Inf, -Inf, Inf]);
    end
    xlabel('t(s)');
    
    figure;
    ylabels = {'v_{x}(m/s)', 'v_{y}(m/s)', 'v_{z}(m/s)'};
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawNoisyVel;
    for j = 1:3
        subplot(3,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_carnation(3),'LineWidth',1.5);
        grid on; hold on;
        ylabel(ylabels{j});
        scatter(t(outlier),tmpData(outlier,j),120,Morandi_carnation(1),'LineWidth',1.5);
        axis([-Inf, Inf, -Inf, Inf]);
    end
    xlabel('t(s)');
end
%}

% Get rid of the outlier
%{
for i = 12:12
    dataShow(i).body1rawNoisyFree = dataShow(i).body1rawNoisy;
    outlier = dataShow(i).body1rawNoisyOutlier;
    dataShow(i).body1rawNoisyFree(outlier,:) =...
        (dataShow(i).body1rawNoisy(outlier-1,:) + dataShow(i).body1rawNoisy(outlier+1,:))/2;
    % Show
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawNoisyFree;
    for j = 1:3
        subplot(3,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_carnation(2),'LineWidth',1.5);
        grid on; hold on;
        ylabel(labels{j+4});
        axis([-Inf, Inf, -Inf, Inf]);
    end
    xlabel('t(s)');
end
%}
