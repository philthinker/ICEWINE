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
