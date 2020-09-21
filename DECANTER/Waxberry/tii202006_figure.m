%tii202006_figure
%   Just for figure and exp
%   No more temporary codes and data
%
%   Haopeng Hu
%   2020.08.07 Good Luck
%   All rights reserved

% load('Data\lead0806_test.mat')

%% alpha
%{
figure;
for i = 1:1
    t = (1:size(data_pre_p_test(i).alpha,2))*0.001 - 0.001;
    plot(t,data_pre_p_test(i).alpha(1,:),'LineWidth',3.0,'Color',[0.46, 0.81, 0.82]);
    hold on;
    plot(t,data_pre_p_test(i).alpha(2,:),'LineWidth',3.0,'Color',[0.62, 0.53, 0.68]);
    grid on; legend('\alpha^{(1)}','\alpha^{(2)}');
end
%}

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
    policy_pre_quat.plotSphere(Data_pre_q(i).expData_simq,[0.8500,0.3250,0.0980],2.0);
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
ylabels = {'q_{w}','q_{x}','q_{y}','q_{z}'};
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        % Demo
        [tmp_z,tmp_i] = zRegulate(Demos_pre(j).posi(3,:));
        tmp_q = quatRegulate(Demos_pre(j).q);
        plot(tmp_z, tmp_q(i,tmp_i),'Color',[0.4,0.4,0.4],'LineWidth',0.8);
        hold on;
        % Execution
        [tmp_z,tmp_i] = zRegulate(Data_pre_w(j).expData_simp(3,:));
        tmp_q = quatRegulate(Data_pre_q(j).expData_simq);
        plot(tmp_z,tmp_q(i,tmp_i),'Color',[0.85,0.325,0.098],'LineWidth',2.4);
        % Retrieve
        [tmp_z,tmp_i] = zRegulate(Data_pre_q(j).query_q);
        tmp_q = quatRegulate(Data_pre_q(j).expData_q);
        plot(tmp_z,tmp_q(i,tmp_i),'Color',[0.0,0.635,0.908],'LineWidth',1.2);
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

figure;
ylabels = {'q_{w}','q_{x}','q_{y}','q_{z}'};
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
    axis([-Inf,Inf,-Inf,Inf]);
    if i ==4
        xlabel('z(m)');
    end
end
%}

% plot with time input
%{
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
%}

% plot3
%{
figure;
[X,Y,Z] = sphere(20);
mesh(X,Y,Z,'EdgeColor',[0.6,0.6,0.6]);
hold on;
% for i = 1:M
%     policy_pre_quat.plotSphere(Demos_pre(i).q,[0.4,0.4,0.4],1.2);
% end
for i = M+1:M+MG
    policy_pre_quat.plotSphere(Data_pre_q(i).expData_q,[0.0,0.635,0.908],2.0);
end
for i = M+1:M+MG
    policy_pre_quat.plotSphere(Data_pre_q(i).expData_simq,[0.8500,0.3250,0.0980],2.0);
end
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1,1,-1,1,0,1]);
view(25,35);
%}

%% Figure OFK and KFK

% % plot with t
%{
figure;
for i = 1:6
    subplot(6,1,i);
    for j = 1:M
        t = linspace(0,1,size(Demos_cis_ofk{j},2));
        plot(t, Demos_cis_ofk{j}(i,:));
        hold on;
    end
end

figure;
for i = 1:6
    subplot(6,1,i);
    for j = 1:M
        t = linspace(0,1,size(Demos_cis_kfk{j},2));
        plot(t, Demos_cis_kfk{j}(i,:));
        hold on;
    end
end
%}

% % plot with z
%{
figure;
ylabels = {'f_{x}(N)','f_{y}(N)','f_{z}(N)','m_{x}(Nm)','m_{y}(Nm)','m_{z}(Nm)'};
for j = 1:M
    [tmp_z,tmp_i] = sort(Demos_cis(j).posi(3,:));
    tmp_w = Demos_cis(j).ofk(:,tmp_i);
    for i = 1:6
        subplot(6,1,i);
        plot(tmp_z,tmp_w(i,:),'Color',[0.6, 0.6, 0.6],'LineWidth',1.5);
        hold on;
        grid on;
        axis([-Inf,0,-Inf,Inf]);
        ylabel(ylabels{i});
    end
    xlabel('z(mm)');
    [tmp_z,tmp_i] = sort(Data_cis(j).query_w);
    tmp_w = Data_cis(j).expData_w(:,tmp_i);
    for i = 1:6
        subplot(6,1,i);
        plot(tmp_z,tmp_w(i,:),'Color',[0.0,0.635,0.908],'LineWidth',2.5);
    end
end
%}


