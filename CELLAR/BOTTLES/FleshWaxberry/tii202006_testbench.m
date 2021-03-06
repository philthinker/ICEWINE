%tii202006_testbench
%   Just for figure test

%% Test ret pre pos
%{
figure;
for i= 1:M
    tmpx = Demos_pre(i).data(2,:);
    tmpy = Demos_pre(i).data(3,:);
    tmpz = Demos_pre(i).data(4,:);
    plot3(tmpx,tmpy,tmpz,'LineWidth',1.2,'Color',[0.6,0.6,0.6]);
    hold on;
    tmpx = Data_pre_w(i).expData_p(1,:);
    tmpy = Data_pre_w(i).expData_p(2,:);
    tmpz = Data_pre_w(i).expData_p(3,:);
    plot3(tmpx,tmpy,tmpz,'b','LineWidth',1.8);
end
grid on; axis equal;
view(128,20);
%}

%% Test ret pre ori
%{
figure;
[X,Y,Z] = sphere(20);
mesh(X,Y,Z,'EdgeColor',[0.6,0.6,0.6]);
% axis([-1.5,1.5,-1.5,1.5,-1.5,1.5]);
hold on;
for i = 1:M
    policy_pre_quat.plotSphere(Demos_pre(i).q,[0.4,0.4,0.4],1.2);
    hold on;
end
for i = 1:M
    policy_pre_quat.plotSphere(Data_pre_q(i).expData_q,[0.0,0.635,0.908],2.0);
end
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1,1,-1,1,0,1]);
view(25,35);

figure;
t = linspace(0,1,size(Data_pre_q(1).query_q,2));
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        plot(t,quatRegulate(Data_pre_q(j).expData_realq(i,:)));
        hold on;
    end
    grid on;
end
%}

%% Test ret pre pos-ori
%{
figure;
for m = 1:M
    tmpx = Data_pre_w(m).expData_simp(1,:);
    tmpy = Data_pre_w(m).expData_simp(2,:);
    tmpz = Data_pre_w(m).expData_simp(3,:);
    plot3(tmpx,tmpy,tmpz,'Color',[0.6,0.6,0.0]);
    hold on;
end
grid on; axis equal;
view(3);

figure;
t = (1:N)*controller_pre.dt - controller_pre.dt;
for m = 1:M
    plot(t,Data_pre_w(m).expData_simp(3,:));
    hold on;
end

figure;
ylabels = {'qw','qx','qy','qz'};
t = (1:N)*controller_pre.dt - controller_pre.dt;
for j = 1:4
    subplot(5,1,j);
    for m = 1:1
        plot(t,Data_pre_q(m).expData_simq(j,:),'r');
        hold on;
        plot(t,Data_pre_q(m).expData_hat_q_c(j,:),'b');
    end
    ylabel(ylabels{j});
end
subplot(5,1,5);
for m = 1:1
    plot(t,Data_pre_w(m).expData_simp(3,:),'r');
    hold on;
    plot(t,Data_pre_w(m).expData_hatp_c(3,:),'b');
end
ylabel('z');
%}

%% Test gen pre pos
%{
figure;
N = 100000;
dt = 0.001;
t = (1:N)*dt - dt;
for i = 1:3
    subplot(3,1,i);
    for j = M+1:M+MG
        plot(t, Data_pre_w(j).expData_simp(i,:));
        hold on;
    end
end

figure;
for i = 1:4
    subplot(4,1,i);
    for j = M+1:M+MG
        plot(t, Data_pre_q(j).expData_simq(i,:));
        hold on;
    end
end
%}

%% FW2 test 

policy_pre_p_test = FWTPGMM2(8,4,2);
policy_pre_p_test = policy_pre_p_test.initGMMTimeBased(Demos_pre);
policy_pre_p_test = policy_pre_p_test.learnGMM(Demos_pre);
N = 1000;
for i = 1:M
    data_pre_p_test(i).query_p = linspace(0,1,N);
    data_pre_p_test(i).query_frame(1).A = Demos_pre(i).A(:,:,1);
    data_pre_p_test(i).query_frame(1).b = Demos_pre(i).b(:,1);
    data_pre_p_test(i).query_frame(2).A = Demos_pre(i).A(:,:,2);
    data_pre_p_test(i).query_frame(2).b = Demos_pre(i).b(:,2);
    [data_pre_p_test(i).expData_p,data_pre_p_test(i).expSigma_p,data_pre_p_test(i).alpha] =...
        policy_pre_p_test.GMR(data_pre_p_test(i).query_p,data_pre_p_test(i).query_frame);
end
figure;
for i = 1:M
    X = Demos_pre(i).data(2,:);
    Y = Demos_pre(i).data(3,:);
    Z = Demos_pre(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6],'LineWidth',1.2);
    hold on;
    X = data_pre_p_test(i).expData_p(1,:);
    Y = data_pre_p_test(i).expData_p(2,:);
    Z = data_pre_p_test(i).expData_p(3,:);
    plot3(X,Y,Z,'b','LineWidth',1.8);
%     legend({'Demonstrated trajectories','Retrieved trajectories'});
end
grid on; axis equal;
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
view(128,20);

figure;
for i = 1:1
    t = (1:size(data_pre_p_test(i).alpha,2))*0.001 - 0.001;
    for j = 1:2
        plot(t,data_pre_p_test(i).alpha(j,:));
        hold on;
    end
    grid on; legend('\alpha^{(1)}','\alpha^{(2)}');
end
