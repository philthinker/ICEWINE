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

%% Retrieve or generalize

% % Init. data storation
%{
MG = 9;

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
for i = 1:M
    Data_pre_w(i).query_p = linspace(0,1,1000);
    Data_pre_w(i).query_frame(1).A = Demos_pre(i).A(:,:,1);
    Data_pre_w(i).query_frame(1).b = Demos_pre(i).b(:,1);
    Data_pre_w(i).query_frame(2).A = Demos_pre(i).A(:,:,2);
    Data_pre_w(i).query_frame(2).b = Demos_pre(i).b(:,2);
    [Data_pre_w(i).expData_p,Data_pre_w(i).expSigma_p,Data_pre_w(i).alpha] =...
        policy_pre_posi_w.GMR(Data_pre_w(i).query_p,Data_pre_w(i).query_frame);
end

figure;
for i = 1:M
    X = Demos_pre(i).data(2,:);
    Y = Demos_pre(i).data(3,:);
    Z = Demos_pre(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6]);
    hold on;
    X = Data_pre_w(i).expData_p(1,:);
    Y = Data_pre_w(i).expData_p(2,:);
    Z = Data_pre_w(i).expData_p(3,:);
    plot3(X,Y,Z,'b');
end
grid on; axis equal;
%}
%{
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
    plot3(X,Y,Z,'r');
    X = Data_pre_w(i).expData_p(1,:);
    Y = Data_pre_w(i).expData_p(2,:);
    Z = Data_pre_w(i).expData_p(3,:);
    plot3(X,Y,Z,'b');
end
grid on; axis equal;
figure;
for i = 1:1
    t = (1:size(Data_pre_w(i).alpha,2))*dt-dt;
    for j = 1:2
        plot(t,Data_pre_w(i).alpha(j,:));
        hold on;
    end
    grid on; legend('alpha 1','alpha 2');
end
%}

% % Retrieve quat

for i = 1:M
    Data_pre(i).query_q = permute(Demos_pre(i).TPData(4,2,:),[1,3,2]);
    [Data_pre(i).expData_eta, Data_pre(i).expSigma_eta] = policy_pre_quat.GMR(Data_pre(i).query_q);
    Data_pre(i).expData_q = policy_pre_quat.expmap(Data_pre(i).expData_eta);
    Data_pre(i).expData_realq = quatProduct(repmat(Data_pre(i).qa,[1,size(Data_pre(i).expData_q,2)]), Data_pre(i).expData_q);
end

% for i = 1:M
%     Data_pre_q(i).query_q = Data_pre(i).query_q;
%     Data_pre_q(i).qa = Data_pre(i).qa;
%     Data_pre_q(i).expData_eta = Data_pre(i).expSigma_eta;
%     Data_pre_q(i).expSigma_eta = Data_pre(i).expSigma_eta;
%     Data_pre_q(i).expData_q = quatRegulate( Data_pre(i).expData_q );
%     Data_pre_q(i).expData_realq = quatRegulate( Data_pre(i).expData_realq );
% end

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

figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        t = (1:size(Data_pre_q(j).expData_q,2))*dt-dt;
        plot(t,Data_pre(i).expData_q(i,:),'Color',[0.0,0.0,0.9]); hold on;
        t = (1:size(Demos_pre(j).q,2))*dt-dt;
        tmpDemo_q = quatRegulate( Demos_pre(j).q );
        plot(t,tmpDemo_q(i,:),'Color',[0.6,0.6,0.6]);
    end
    grid on;
end

figure;
dt = 0.1;
for i = 1:4
    subplot(4,1,i);
    for j = 1:1
        t = (1:size(Data_pre_q(j).expData_realq,2))*dt-dt;
        plot(t,Data_pre_q(j).expData_realq(i,:)); hold on;
    end
    grid on;
end
%}
