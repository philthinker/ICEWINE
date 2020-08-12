%tii202006_figure
%   Just for figure and exp
%   No more temporary codes and data
%
%   Haopeng Hu
%   2020.08.07 Good Luck
%   All rights reserved

% load('Data\lead0806_test.mat')

%% Retrieve pre q

% plot3
%{
figure;
[X,Y,Z] = sphere(20);
mesh(X,Y,Z,'EdgeColor',[0.6,0.6,0.6]);
hold on;
for i = 1:M
    policy_pre_quat.plotSphere(Demos_pre(i).q,[0.4,0.4,0.4],1.2);
end
for i = 1:M
    policy_pre_quat.plotSphere(Data_pre_q(i).expData_q,[0.0,0.635,0.908],2.0);
end
for i = 1:M
    policy_pre_quat.plotSphere(Data_pre_q(i).exeData_q,[0.8500,0.3250,0.0980],2.0);
end
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1,1,-1,1,0,1]);
view(25,35);
%}
% plot
%{
figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        tmpN = size(Demos_pre(j).q,2);
        tmpt = linspace(0,1,tmpN);
        % Demo
        plot(tmpt, quatRegulate( Demos_pre(j).q(i,:) ),'Color',[0.4,0.4,0.4],'LineWidth',0.8);
        hold on;
        % Retrieve
        tmpN = size(Data_pre_q(j).expData_q,2);
        tmpt = linspace(0,1,tmpN);
        plot(tmpt, quatRegulate(Data_pre_q(j).expData_q(i,:)),'Color',[0.0,0.635,0.908],'LineWidth',1.0);
        % Execution
        tmpN = size(Data_pre_q(j).exeData_q,2);
        tmpt = linspace(0,1,tmpN);
        plot(tmpt, quatRegulate(Data_pre_q(j).exeData_q(i,:)),'Color',[0.85,0.325,0.098],'LineWidth',1.2);
    end
    grid on;
end
%}
% plot with z input
%{
figure;
ylabels = {'qw','qx','qy','qz'};
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        % Demo
        [tmp_z,tmp_i] = zRegulate(Demos_pre(j).posi(3,:));
        tmp_q = quatRegulate(Demos_pre(j).q);
        plot(tmp_z, tmp_q(i,tmp_i),'Color',[0.4,0.4,0.4],'LineWidth',0.8);
        hold on;
        % Retrieve
        [tmp_z,tmp_i] = zRegulate(Data_pre_q(j).query_q);
        tmp_q = quatRegulate(Data_pre_q(j).expData_q);
        plot(tmp_z,tmp_q(i,tmp_i),'Color',[0.0,0.635,0.908],'LineWidth',1.6);
        % Execution
        [tmp_z,tmp_i] = zRegulate(Data_pre_w(j).expData_simp(3,:));
        tmp_q = quatRegulate(Data_pre_q(j).expData_simq);
        plot(tmp_z,tmp_q(i,tmp_i),'Color',[0.85,0.325,0.098],'LineWidth',1.5);
    end
    grid on;
    ylabel(ylabels{i});
    axis([-0.8,0.0,-Inf,Inf]);
    if i ==4
        xlabel('z(m)');
    end
end
%}
% scatter with z input
%{
figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        % Demo
        tmp_z = Demos_pre(j).posi(3,:);
        scatter(tmp_z, quatRegulate( Demos_pre(j).q(i,:) ),'filled');
        hold on;
        % Retrieve
%         plot(quatRegulate(Data_pre_q(j).expData_q(i,:)),'Color',[0.0,0.635,0.908],'LineWidth',1.0);
%         % Execution
%         plot(quatRegulate(Data_pre_q(j).exeData_q(i,:)),'Color',[0.85,0.325,0.098],'LineWidth',1.2);
    end
    grid on;
end
%}

%% Generate pre q

% plot with z input
%{
figure;
ylabels = {'qw','qx','qy','qz'};
for i = 1:4
    subplot(4,1,i);
    for j = M+1:M+MG
        % Retrieve
        [tmp_z,tmp_i] = zRegulate(Data_pre_q(j).query_q);
        tmp_q = quatRegulate(Data_pre_q(j).expData_q);
        plot(tmp_z,tmp_q(i,tmp_i),'Color',[0.0,0.635,0.908],'LineWidth',1.6);
        hold on;
        % Execution
%         [tmp_z,tmp_i] = zRegulate(Data_pre_w(j).expData_simp(3,:));
%         tmp_q = quatRegulate(Data_pre_q(j).expData_simq);
        tmp_z = Data_pre_w(j).expData_simp(3,:);
        tmp_i = (1:N);
        tmp_q = Data_pre_q(j).expData_simq;
        plot(tmp_z,tmp_q(i,tmp_i),'Color',[0.85,0.325,0.098],'LineWidth',1.5);
    end
    grid on;
    ylabel(ylabels{i});
%      axis([-Inf,Inf,-Inf,Inf]);
    if i ==4
        xlabel('z(m)');
    end
end
%}

% plot with time input
figure;
t = (1:N)*dt - dt;
ylabels = {'qw','qx','qy','qz'};
for i = 1:4
    subplot(4,1,i);
    for j = M+1:M+MG
        % plot the reference
        plot(t,Data_pre_q(j).expData_hat_q_c(i,:),'Color',[0.0,0.635,0.908],'LineWidth',0.8);
        hold on;
        % plot the simq
        plot(t,Data_pre_q(j).expData_simq(i,:),'Color',[0.85,0.325,0.098],'LineWidth',1.2);
    end
    grid on;
    ylabel(ylabels{i});
    if i == 4
        xlabel('t(s)');
    end
    axis([0.0, 50, -Inf, Inf]);
end
