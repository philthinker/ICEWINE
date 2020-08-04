%tii202006_PolicyPreQuat

% % load('Data\lead0726.mat');

%% Init policy and demo data

policy_pre_quat_xyz = QGMMZero(6,7,[1;0;0;0]);
policy_pre_quat_xyz = policy_pre_quat_xyz.setGMRInOut([1,2,3]');

%{
Demos_pre_eta_xyz = cell(1,M);
for i = 1:M
    tmpN = size(Demos_pre(i).q,2);
    Demos_pre_eta_xyz{i} = zeros(6,tmpN);
    Demos_pre_eta_xyz{i}(4:6,:) = policy_pre_quat_xyz.logmap( Demos_pre(i).q );
    Demos_pre_eta_xyz{i}(1:3,:) = Demos_pre(i).posi;
end
%}
%{
Data_pre_q_xyz = [];
Data_pre_q_xyz.query_q = [];
Data_pre_q_xyz.expData_eta = [];
Data_pre_q_xyz.expSigma_eta = [];
Data_pre_q_xyz.expData_q = [];
Data_pre_q_xyz.expData_realq = [];
Data_pre_q_xyz.qa = [];
Data_pre_q_xyz.realqPlus = [];
Data_pre_q_xyz = repmat(Data_pre_q_xyz,[1,M+MG]);
%}

%% Init ret. and gen. data storation

%{
for i = 1:M+MG
    Data_pre_q_xyz(i).qa = Data_pre_q(i).qa;
    tmpR = Data_pre_w(i).query_frame(2).A(2:4,2:4);
    tmpp = Data_pre_w(i).query_frame(2).b(2:4);
    Data_pre_q_xyz(i).query_q = tmpR' * (Data_pre_w(i).expData_p - tmpp);
end
figure;
for i = 1:M+MG
    tmpX = Data_pre_q_xyz(i).query_q(1,:);
    tmpY = Data_pre_q_xyz(i).query_q(2,:);
    tmpZ = Data_pre_q_xyz(i).query_q(3,:);
    plot3(tmpX,tmpY,tmpZ);
    hold on;
end
grid on; axis equal;
view(3);
%}

%% Learn the policy

policy_pre_quat_xyz = policy_pre_quat_xyz.initGMMKMeans(policy_pre_quat_xyz.dataRegulate( Demos_pre_eta_xyz ));
policy_pre_quat_xyz = policy_pre_quat_xyz.learnGMM(policy_pre_quat_xyz.dataRegulate( Demos_pre_eta_xyz ));

%% Retrieve and generate trajectories

% % GMR

for i = 1:M+MG
    [Data_pre_q_xyz(i).expData_eta,Data_pre_q_xyz(i).expSigma_eta] = policy_pre_quat_xyz.GMR(Data_pre_q_xyz(i).query_q);
    Data_pre_q_xyz(i).expData_q = quatRegulate( policy_pre_quat_xyz.expmap(Data_pre_q_xyz(i).expData_eta) );
%     Data_pre_q_xyz(i).expData_realq = quatProduct(repmat(Data_pre_q_xyz(i).qa,[1,size(Data_pre_q_xyz(i).expData_q,2)]), Data_pre_q_xyz(i).expData_q);
end
%}

% % Add the compensation
%{
for i = 1:M+MG
    Data_pre_q_xyz(i).realqPlus = Data_pre_q_xyz(i).expData_realq;
    Data_pre_q_xyz(i).realqPlus(:,end+1) = Data_pre_q_xyz(i).qa;
end
%}

%% Figure

% % Demo and Ret on sphere
% [0.0,0.635,0.908]

figure;
[X,Y,Z] = sphere(20);
mesh(X,Y,Z,'EdgeColor',[0.6,0.6,0.6]);
% axis([-1.5,1.5,-1.5,1.5,-1.5,1.5]);
hold on;
for i = 1:M
    policy_pre_quat_xyz.plotSphere(Demos_pre(i).q,[0.4,0.4,0.4],1.2);
    hold on;
    policy_pre_quat_xyz.plotSphere(Data_pre_q_xyz(i).expData_q,[0.0,0.635,0.908],2.0);
end
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1,1,-1,1,0,1]);
view(25,35);

% % Demo and Gen on sphere

figure;
[X,Y,Z] = sphere(20);
mesh(X,Y,Z,'EdgeColor',[0.6,0.6,0.6]);
% axis([-1.5,1.5,-1.5,1.5,-1.5,1.5]);
hold on;
for i = 9:9
    policy_pre_quat_xyz.plotSphere(Data_pre_q_xyz(M+i).expData_q,0,2.0,0);
    hold on;
end
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1,1,-1,1,0,1]);
view(25,35);
