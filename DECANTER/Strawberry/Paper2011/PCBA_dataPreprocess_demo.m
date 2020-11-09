%PCBA_dataPreprocess_demo
%   Reserved for Z. Cao and Z. Zhao's thesis
%
%   Haopeng Hu
%   2020.11.09
%   All rights reserved

% Raw data: 'Data\data0930_honeytea.mat'
% Data: 'Data\PCBA_dataPreprocess.mat'

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
    tmpData = Demos(i).rawData_l;
%     dataShow(i).rawOptiLeft = tmpData;  % qw qx qy qz w y z 
    plot3(tmpData(:,5),tmpData(:,6),tmpData(:,7),'Color',Morandi_hydrangea(i),'LineWidth',2.0);
    hold on;
    tmpData = Demos(i).rawData_r;
%     dataShow(i).rawOptiRight = tmpData;
    plot3(tmpData(:,5),tmpData(:,6),tmpData(:,7),'Color',Morandi_hydrangea(i),'LineWidth',2.0);
end
grid on; axis equal; xlabel(labels{5}); ylabel(labels{6}); zlabel(labels{7});
view(3);
%}

% Plot
%{
for i = 1:1
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1raw;
    for j = 1:7
        subplot(7,1,j);
        plot(t,tmpData(:,j),'Color',Morandi_hydrangea(i),'LineWidth',1.5);
        grid on;
        ylabel(labels{j});
    end
    xlabel('t(s)');
end
%}

%% Gapped data

%{
% [dataShow(1).body1rawGapped, ~] = dataGap([1.950, 2.442], dataShow(1).t, dataShow(1).body1raw);
% [dataShow(1).body1rawGapped, tmpLogicID] = dataGap([0.331, 0.588], dataShow(1).t, dataShow(1).body1rawGapped);
% dataShow(1).body1rawGapped(:,1:4) = quatRegulate(dataShow(1).body1rawGapped(:,1:4), true);
for i = 1:1
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
for i = 1:1
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
    dataShow(i).TEtaP = tmpTEtaP;
end
%}

% GPR
%{
for i = 1:1
    gp = GPZero(dataShow(i).BodyTEtaP1);
    sigma = 1e0; W = 1e-1;
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
for m = 1:1
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
%             title('sigma = 1e0; W = 1e-1');
        end
    end
    xlabel('t(s)');
end
%}
