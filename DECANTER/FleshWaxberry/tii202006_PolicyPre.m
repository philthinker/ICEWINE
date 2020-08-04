%tii202006_PolicyPre

% policy_pre_posi = TPGMMOne(8,4,2);
% policy_pre_quat = QGMMZero(6,5,[1;0;0;0]);
% policy_pre_posi_w = FWTPGMM(8,4,2);

%% Init. data

% % load(¡®Data\lead0722')

% % Init. data for TPGMM
%{
Demos_pre = [];
Demos_pre.data = [];
Demos_pre.A = [];
Demos_pre.b = [];
Demos_pre.TPData = [];
Demos_pre = repmat(Demos_pre,[1,M]);
for i = 1:M
    Demos_pre(i) = policy_pre_posi.TPDemoConstruct(Demo(i).pre.p_dtw',Demo(i).pre.R,Demo(i).pre.p,true);
end
%}

% % Init. data for QGMM
%{
for i = 1:M
    Demos_pre(i).R = Demo(i).pre.R;
    Demos_pre(i).p = Demo(i).pre.p;
    Demos_pre(i).qa = Demo(i).pre.pq(end,4:7)';
    Demos_pre(i).posi = Demo(i).pre.tp2p;
    Demos_pre(i).q = Demo(i).pre.tp2q;
end

Demos_pre_eta = cell(1,M);
for i = 1:M
    tmpN = size(Demos_pre(i).q,2);
    Demos_pre_eta{i} = zeros(4,tmpN);
    Demos_pre_eta{i}(2:4,:) = policy_pre_quat.logmap( Demos_pre(i).q );
    Demos_pre_eta{i}(1,:) = Demos_pre(i).posi(3,:);
end
%}

%% Learn TP-GMM and QGMM

% % Learn policy position
%{
policy_pre_posi = policy_pre_posi.initGMMTimeBased(Demos_pre);
policy_pre_posi = policy_pre_posi.learnGMM(Demos_pre);
%}
%{
policy_pre_posi_w = policy_pre_posi_w.initGMMTimeBased(Demos_pre);
policy_pre_posi_w = policy_pre_posi_w.learnGMM(Demos_pre);
%}

% % learn policy quaternion
%{
policy_pre_quat = policy_pre_quat.initGMMKMeans(policy_pre_quat.dataRegulate(Demos_pre_eta));
policy_pre_quat = policy_pre_quat.learnGMM(policy_pre_quat.dataRegulate(Demos_pre_eta));
%}

%% Retrieve

% % Init. data storation
%{
MG = 9;
N = 1000;

Data_pre = [];
Data_pre.query_p = [];
query_frame = [];
query_frame.A = [];
query_frame.b = [];
Data_pre.query_frame = repmat(query_frame,[1,2]);
Data_pre.expData_p = [];
Data_pre.expSigma_p = [];
Data_pre.query_q = [];
Data_pre.expData_eta = [];
Data_pre.expSigma_eta = [];
Data_pre.expData_q = [];
Data_pre.expData_realq = [];
Data_pre.qa = [];

Data_pre= repmat(Data_pre,[1,M+MG]);
%}
%{
Data_pre_w = [];
Data_pre_w.query_p = [];
query_frame = [];
query_frame.A = [];
query_frame.b = [];
Data_pre_w.query_frame = repmat(query_frame,[1,2]);
Data_pre_w.expData_p = [];
Data_pre_w.expSigma_p = [];
Data_pre_w.alpha = [];
Data_pre_w.expData_pPlus = [];

Data_pre_w = repmat(Data_pre_w,[1,M+MG]);
%}
%{
Data_pre_q = [];
Data_pre_q.query_q = [];
Data_pre_q.expData_eta = [];
Data_pre_q.expSigma_eta = [];
Data_pre_q.expData_q = [];
Data_pre_q.expData_realq = [];
Data_pre_q.qa = [];

Data_pre_q = repmat(Data_pre_q,[1,M+MG]);
%}

% % Retrieve position
%{
for i = 1:M
    Data_pre(i).query_p = linspace(0,1,200);
    Data_pre(i).query_frame(1).A = Demos_pre(i).A(:,:,1);
    Data_pre(i).query_frame(1).b = Demos_pre(i).b(:,1);
    Data_pre(i).query_frame(2).A = Demos_pre(i).A(:,:,2);
    Data_pre(i).query_frame(2).b = Demos_pre(i).b(:,2);
    [Data_pre(i).expData_p,Data_pre(i).expSigma_p] = policy_pre_posi.GMR(Data_pre(i).query_p,Data_pre(i).query_frame);
end

figure;
for i = 1:M
    X = Demos_pre(i).data(2,:);
    Y = Demos_pre(i).data(3,:);
    Z = Demos_pre(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6]);
    hold on;
    X = Data_pre(i).expData_p(1,:);
    Y = Data_pre(i).expData_p(2,:);
    Z = Data_pre(i).expData_p(3,:);
    plot3(X,Y,Z,'b');
end
grid on; axis equal;
%}
%{
N = 20000;
for i = 1:M
    Data_pre_w(i).query_p = linspace(0,1,N);
    Data_pre_w(i).query_frame(1).A = Demos_pre(i).A(:,:,1);
    Data_pre_w(i).query_frame(1).b = Demos_pre(i).b(:,1);
    Data_pre_w(i).query_frame(2).A = Demos_pre(i).A(:,:,2);
    Data_pre_w(i).query_frame(2).b = Demos_pre(i).b(:,2);
    [Data_pre_w(i).expData_p,Data_pre_w(i).expSigma_p,Data_pre_w(i).alpha] =...
        policy_pre_posi_w.GMR(Data_pre_w(i).query_p,Data_pre_w(i).query_frame);
end
%}
%{
figure;
for i = 1:M
    X = Demos_pre(i).data(2,:);
    Y = Demos_pre(i).data(3,:);
    Z = Demos_pre(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6],'LineWidth',1.2);
    hold on;
    X = Data_pre_w(i).expData_p(1,:);
    Y = Data_pre_w(i).expData_p(2,:);
    Z = Data_pre_w(i).expData_p(3,:);
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
    t = (1:size(Data_pre_w(i).alpha,2))*0.001 - 0.001;
    for j = 1:2
        plot(t,Data_pre_w(i).alpha(j,:));
        hold on;
    end
    grid on; legend('\alpha^{(1)}','\alpha^{(2)}');
end
%}

% % Retrieve quat
%{
% for i = 1:M
%     Data_pre(i).query_q = permute(Demos_pre(i).TPData(4,2,:),[1,3,2]);
%     [Data_pre(i).expData_eta, Data_pre(i).expSigma_eta] = policy_pre_quat.GMR(Data_pre(i).query_q);
%     Data_pre(i).expData_q = policy_pre_quat.expmap(Data_pre(i).expData_eta);
%     Data_pre(i).expData_realq = quatProduct(repmat(Data_pre(i).qa,[1,size(Data_pre(i).expData_q,2)]), Data_pre(i).expData_q);
% end

for i = 1:M
    % Generate query
    tmpDataPre = Data_pre_w(i);
    tmpFrame = tmpDataPre.query_frame(2);
    tmpTPData_p = zeros(3,N);
    for j = 1:N
        tmpTPData_p(:,j) = (tmpFrame.A(2:4,2:4))' * ( tmpDataPre.expData_p(:,j) - tmpFrame.b(2:4) );
    end
    Data_pre_q(i).query_q = tmpTPData_p(3,:);
    Data_pre_q(i).qa = Data_pre(i).qa;
    [Data_pre_q(i).expData_eta, Data_pre_q(i).expSigma_eta] = policy_pre_quat.GMR(Data_pre_q(i).query_q);
    Data_pre_q(i).expData_q = quatRegulate( policy_pre_quat.expmap(Data_pre_q(i).expData_eta) );
    Data_pre_q(i).expData_realq = quatRegulate( quatProduct(...
        repmat(Data_pre_q(i).qa,[1,N]),...
        Data_pre_q(i).expData_q) );
end
%}

%{
% figure;
% for i = 1:M
%     X = Data_pre(i).expData_eta(1,:);
%     Y = Data_pre(i).expData_eta(2,:);
%     Z = Data_pre(i).expData_eta(3,:);
%     plot3(X,Y,Z);
%     hold on;
% end
% grid on; axis equal;

% figure;
% dt = 0.1;
% for i = 1:4
%     subplot(4,1,i);
%     for j = 1:M
%         t = (1:size(Data_pre(j).expData_q,2))*dt-dt;
%         plot(t,Data_pre(i).expData_q(i,:),'Color',[0.0,0.0,0.9]); hold on;
%         t = (1:size(Demos_pre(j).q,2))*dt-dt;
%         plot(t,Demos_pre(j).q(i,:),'Color',[0.6,0.6,0.6]);
%     end
%     grid on;
% end
% 
% figure;
% dt = 0.1;
% for i = 1:4
%     subplot(4,1,i);
%     for j = 1:1
%         t = (1:size(Data_pre(j).expData_realq,2))*dt-dt;
%         plot(t,Data_pre(j).expData_realq(i,:)); hold on;
%     end
%     grid on;
% end

% figure;
% for i = 1:4
%     subplot(4,1,i);
%     for j = 1:M
%         t = linspace(0,1,N);
%         plot(t,Data_pre_q(i).expData_q(i,:),'Color',[0.0,0.0,0.9]); hold on;
%         t = linspace(0,1,size(Demos_pre(j).q,2));
%         tmpDemo_q = quatRegulate( Demos_pre(j).q );
%         plot(t,tmpDemo_q(i,:),'Color',[0.6,0.6,0.6]);
%     end
%     grid on;
% end

% figure;
% dt = 0.1;
% for i = 1:4
%     subplot(4,1,i);
%     for j = 1:1
%         t = (1:size(Data_pre_q(j).expData_realq,2))*dt-dt;
%         plot(t,Data_pre_q(j).expData_realq(i,:)); hold on;
%     end
%     grid on;
% end

figure;
[X,Y,Z] = sphere(20);
mesh(X,Y,Z,'EdgeColor',[0.6,0.6,0.6]);
% axis([-1.5,1.5,-1.5,1.5,-1.5,1.5]);
hold on;
for i = 1:M
    policy_pre_quat.plotSphere(Demos_pre(i).q,[0.4,0.4,0.4],1.2);
    hold on;
    policy_pre_quat.plotSphere(Data_pre_q(i).expData_q,[0.0,0.635,0.908],2.0);
end
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1,1,-1,1,0,1]);
view(25,35);
%}

% % Add the terminal state
%{
for i = 1:M
    Data_pre_w(i).expData_pPlus = Data_pre_w(i).expData_p;
    Data_pre_w(i).expData_pPlus(:,end+1) = Demos_pre(i).data(2:4,end);
    Data_pre_q(i).expData_realqPlus = Data_pre_q(i).expData_realq;
    Data_pre_q(i).expData_realqPlus(:,end+1) = Data_pre_q(i).qa;
end
%}

%% Generalization init

% % Init. data
%{
Gen_pre = [];
Gen_pre.init_otee = [];
Gen_pre.final_otee = [];
Gen_pre.init_se3 = [];
Gen_pre.final_se3 = [];
Gen_pre.init_p = [];
Gen_pre.final_p = [];
Gen_pre.init_q = [];
Gen_pre.final_q = [];

Gen_pre =repmat(Gen_pre,[1,MG]);
%}
%{
for i = 1:MG
    % Read data
    Gen_pre(i).init_otee = readmatrix(strcat('DECANTER\FleshWaxberry\Data\07-29\pre_gen\gen0',...
                                                                    int2str(i), '_OTEE.csv'));
    % Get rid of the last column
    Gen_pre(i).init_otee = Gen_pre(i).init_otee(:,1:end-1);
    Gen_pre(i).init_se3 = fold2SE3(Gen_pre(i).init_otee);
    Gen_pre(i).init_p = Gen_pre(i).init_otee(:,13:15)';
    Gen_pre(i).init_q = quatRegulate(tform2quat(Gen_pre(i).init_se3)');
    Gen_pre(i).final_otee = readmatrix(strcat('DECANTER\FleshWaxberry\Data\07-29\pre_gen\ret0',...
                                                                    int2str(1), '_dummy.csv'));
    % Get rid of the last column
    Gen_pre(i).final_otee = Gen_pre(i).final_otee(:,1:end-1);
    Gen_pre(i).final_se3 = fold2SE3(Gen_pre(i).final_otee);
    Gen_pre(i).final_p = Gen_pre(i).final_otee(:,13:15)';
    Gen_pre(i).final_q = quatRegulate(tform2quat(Gen_pre(i).final_se3)');
end
%}

% % Generate query of p
%{
for i = M+1 : M+MG
    Data_pre_w(i).query_p = linspace(0,1,N);
    tmpA1 = eye(4); tmpA1(2:4,2:4) = Gen_pre(i-M).init_se3(1:3,1:3);
    tmpA2 = eye(4); tmpA2(2:4,2:4) = Gen_pre(i-M).final_se3(1:3,1:3);
    Data_pre_w(i).query_frame(1).A = tmpA1;
    Data_pre_w(i).query_frame(2).A = tmpA2;
    tmpb1 = zeros(4,1); tmpb1(2:4,1) = Gen_pre(i-M).init_p;
    tmpb2 = zeros(4,1); tmpb2(2:4,1) = Gen_pre(i-M).final_p;
    Data_pre_w(i).query_frame(1).b = tmpb1;
    Data_pre_w(i).query_frame(2).b = tmpb2;
end
%}

% % Generalize p
%{
% for i = M+1 : M+ MG
%     [Data_pre_w(i).expData_p, Data_pre_w(i).expSigma_p,Data_pre_w(i).alpha] =...
%         policy_pre_posi_w.GMR(Data_pre_w(i).query_p,Data_pre_w(i).query_frame);
% end
%}
%{
figure;
for i = 1:M
    X = Demos_pre(i).data(2,:);
    Y = Demos_pre(i).data(3,:);
    Z = Demos_pre(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6],'LineWidth',1.2);
    hold on;
%     X = Data_pre_w(i).expData_p(1,:);
%     Y = Data_pre_w(i).expData_p(2,:);
%     Z = Data_pre_w(i).expData_p(3,:);
%     plot3(X,Y,Z,'b');
end
for i = M+1:M+MG
    X = Data_pre_w(i).expData_p(1,:);
    Y = Data_pre_w(i).expData_p(2,:);
    Z = Data_pre_w(i).expData_p(3,:);
    plot3(X,Y,Z,'r','LineWidth',1.8);
end
grid on; axis equal;
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
view(128,20);

figure;
for i = 1:M+MG
    t = (1:size(Data_pre_w(i).alpha,2))*dt-dt;
    for j = 1:2
        plot(t,Data_pre_w(i).alpha(j,:));
        hold on;
    end
    grid on; legend('alpha 1','alpha 2');
end
%}

% % Generate query of q
%{
for i = M+1:M+MG
    Data_pre_q(i).qa = Gen_pre(i-M).final_q;
    tmp_quer_q = (Data_pre_w(i).query_frame(2).A(2:4,2:4))' * (Data_pre_w(i).expData_p - Data_pre_w(i).query_frame(2).b(2:4));
    Data_pre_q(i).query_q = tmp_quer_q(3,:);
end
%}

% % Generalize q
%{
for i = M+1 : M+MG
    [Data_pre_q(i).expData_eta, Data_pre_q(i).expSigma_eta] = policy_pre_quat.GMR(Data_pre_q(i).query_q);
    Data_pre_q(i).expData_q = quatRegulate(policy_pre_quat.expmap(Data_pre_q(i).expData_eta));
    Data_pre_q(i).expData_realq = quatRegulate( quatProduct(...
        repmat(Data_pre_q(i).qa,[1,N]),...
        Data_pre_q(i).expData_q) );
end
%}

% % Compensation
%{
for i = M+1:M+MG
    Data_pre_w(i).expData_pPlus = Data_pre_w(i).expData_p;
    Data_pre_w(i).expData_pPlus(:,end+1) = Gen_pre(i-M).final_p;
    Data_pre_q(i).expData_realqPlus = Data_pre_q(i).expData_realq;
    Data_pre_q(i).expData_realqPlus(:,end+1) = Data_pre_q(i).qa;
end
%}

