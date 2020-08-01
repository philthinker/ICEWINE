%tii202006_DataGen
%   Generate the data used in real experiments
%   

%% Policy Pre Retrieve P

% load('Data/lead0722')

% Generate origin exp data
%{
fileName = 'ret_pre_';
for i = 1:M
    csvwrite( strcat(fileName,'p_',int2str(i),'.csv'),toPandaCarte(Data_pre_w(i).expData_p));
    csvwrite( strcat(fileName,'q_',int2str(i),'.csv'),toPandaCarte(Data_pre_q(i).expData_realq));
    csvwrite( strcat(fileName,'pPlus_',int2str(i),'.csv'),toPandaCarte(Data_pre_w(i).expData_pPlus));
    csvwrite( strcat(fileName,'qPlus_',int2str(i),'.csv'),toPandaCarte(Data_pre_q(i).expData_realqPlus));
    tmpSO3 = quat2rotm(Data_pre_q(i).expData_realqPlus');
    tmpSE3 = SO3P2SE3(tmpSO3, Data_pre_w(i).expData_pPlus);
    csvwrite( strcat(fileName,'se3_',int2str(i),'.csv'),flattenSE3(tmpSE3));
end
%}

%{
fileName = 'gen_pre_';
for i = M+1:M+MG
    csvwrite( strcat(fileName,'p_',int2str(i),'.csv'),toPandaCarte(Data_pre_w(i).expData_p));
    csvwrite( strcat(fileName,'q_',int2str(i),'.csv'),toPandaCarte(Data_pre_q(i).expData_realq));
    csvwrite( strcat(fileName,'pPlus_',int2str(i),'.csv'),toPandaCarte(Data_pre_w(i).expData_pPlus));
    csvwrite( strcat(fileName,'qPlus_',int2str(i),'.csv'),toPandaCarte(Data_pre_q(i).expData_realqPlus));
    tmpSO3 = quat2rotm(Data_pre_q(i).expData_realqPlus');
    tmpSE3 = SO3P2SE3(tmpSO3, Data_pre_w(i).expData_pPlus);
    csvwrite( strcat(fileName,'se3_',int2str(i),'.csv'),flattenSE3(tmpSE3));
end
%}

% % Comparison
%{
figure;
for i = 1:M
    tmpX = Demos_pre(i).data(2,:);
    tmpY = Demos_pre(i).data(3,:);
    tmpZ = Demos_pre(i).data(4,:);
    plot3(tmpX,tmpY,tmpZ,'Color',[0.6, 0.6, 0.6]);
    hold on;
end
grid on; axis equal;

tmpX = Data_pre_w(1).expData_pPlus(1,:);
tmpY = Data_pre_w(1).expData_pPlus(2,:);
tmpZ = Data_pre_w(1).expData_pPlus(3,:);
plot3(tmpX,tmpY,tmpZ,'Color',[0.0, 0.0, 0.6]);

tmpX = testpre01(:,20);
tmpY = testpre01(:,21);
tmpZ = testpre01(:,22);
plot3(tmpX,tmpY,tmpZ,'Color',[0.6, 0.0, 0.0]);

figure;
t = linspace(0,1,size(testpre01,1));
for j = 1:7
    subplot(7,1,j);
    plot(t,testpre01(:,j));
end

figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        tmpN = size(Demos_pre(j).q,2);
        t = (1:tmpN)*dt - dt;
        tmpq = quatRegulate(Demos_pre(j).q);
        plot(t, tmpq(i,:),'Color',[0.6, 0.6, 0.6]);
        hold on;
    end
    grid on;
end
%}

%% Policy Cis Retrieve P,Q,W

% load('Data/lead0724')

% Generate origin exp data
%{
fileName = 'ret_cis_';
for i = 1:M+1
    csvwrite( strcat(fileName,'p_',int2str(i),'.csv'),toPandaCarte(Data_cis(i).expData_p/1000)); % Never forget the unit issue
    csvwrite( strcat(fileName,'q_',int2str(i),'.csv'),toPandaCarte(Data_cis(i).expData_realq));
    csvwrite( strcat(fileName,'w_',int2str(i),'.csv'),toPandaCarte(Data_cis(i).expData_w));
    tmpSO3 = quat2rotm(Data_cis(i).expData_realq');
    tmpSE3 = SO3P2SE3(tmpSO3, Data_cis(i).expData_p/1000);
    csvwrite( strcat(fileName,'se3_',int2str(i),'.csv'),flattenSE3(tmpSE3));
end
%}