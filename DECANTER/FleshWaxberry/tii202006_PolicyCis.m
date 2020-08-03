%tii202006_PolicyCis

% policy_cis_posi = TPGMMOne(4,4,2);
% % policy_cis_posi = TPGMMOne(6,4,2);
% policy_cis_quat = QGMMZero(6,5,[1;0;0;0]);
% policy_cis_wren = GMMOne(6,7);

%% Data Init.

% % load(¡®Data\lead0724')

% % Init. data for TP-GMM
%{
Demos_cis = [];
Demos_cis.data = [];
Demos_cis.A = [];
Demos_cis.b = [];
Demos_cis.TPData = [];
Demos_cis = repmat(Demos_cis,[1,M]);
for i = 1:M
    Demos_cis(i) = policy_cis_posi.TPDemoConstruct((Demo(i).cis.p_dtw')*1000, Demo(i).cis.R, Demo(i).cis.p*1000, true);
end
%}

% % Init. data for QGMM
%{
for i = 1:M
    Demos_cis(i).R = Demo(i).cis.R;
    Demos_cis(i).p = Demo(i).cis.p;
    Demos_cis(i).qa = Demo(i).cis.pq(end,4:7)';
    Demos_cis(i).posi = Demo(i).cis.tp2p*1000;
    Demos_cis(i).q = Demo(i).cis.tp2q;
end

Demos_cis_eta = cell(1,M);
for i = 1:M
    tmpN = size(Demos_cis(i).q,2);
    Demos_cis_eta{i} = zeros(4,tmpN);
    Demos_cis_eta{i}(2:4,:) = policy_cis_quat.logmap( Demos_cis(i).q );
    Demos_cis_eta{i}(1,:) = Demos_cis(i).posi(3,:);
end
%}

% % Init. data for GMM
%{
for i = 1:M
    Demos_cis(i).ofk = Demo(i).cis.ofk';
    Demos_cis(i).kfk = Demo(i).cis.kfk';
end

Demos_cis_kfk = cell(1,M);
Demos_cis_ofk = cell(1,M);
for i =1:M
    tmpN = size(Demos_cis(i).kfk,2);
    Demos_cis_kfk{i} = zeros(7,tmpN);
    Demos_cis_ofk{i} = zeros(7,tmpN);
    Demos_cis_kfk{i}(2:7,:) = Demos_cis(i).kfk;
    Demos_cis_ofk{i}(2:7,:) = Demos_cis(i).ofk;
    Demos_cis_kfk{i}(1,:) = Demos_cis(i).posi(3,:);
    Demos_cis_ofk{i}(1,:) = Demos_cis(i).posi(3,:);
end
%}

%% Learn TP-GMM, QGMM, GMM

% % Learn policy position

% % Note that the unit is mm rather than m
% policy_cis_posi = policy_cis_posi.initGMMTimeBased(Demos_cis);
% policy_cis_posi = policy_cis_posi.learnGMM(Demos_cis);

% % Learn policy quaternion
% policy_cis_quat = policy_cis_quat.initGMMKMeans(policy_cis_quat.dataRegulate(Demos_cis_eta));
% policy_cis_quat = policy_cis_quat.learnGMM(policy_cis_quat.dataRegulate(Demos_cis_eta));

% % Learn policy wrench
% policy_cis_wren = policy_cis_wren.initGMMKMeans(policy_cis_wren.dataRegulate(Demos_cis_ofk));
% policy_cis_wren = policy_cis_wren.learnGMM(policy_cis_wren.dataRegulate(Demos_cis_ofk));

%% Retrieve

% % Init. data storation
%{
MG = 9;

Data_cis = [];
Data_cis.query_p = [];
query_frame = [];
query_frame.A = [];
query_frame.b = [];
Data_cis.query_frame = repmat(query_frame,[1,2]);
Data_cis.expData_p = [];
Data_cis.expSigma_p = [];
Data_cis.query_q = [];
Data_cis.expData_eta = [];
Data_cis.expSigma_eta = [];
Data_cis.expData_q = [];
Data_cis.expData_realq = [];
Data_cis.qa = [];
Data_cis.query_w = [];
Data_cis.expData_w = [];
Data_cis.expSigma_w = [];
Data_cis.pre_frame = [];
Data_cis.cis_frame = [];

Data_cis= repmat(Data_cis,[1,M+MG]);
%}

% % Retrieve position data
%{
for i = 1:M
    % Query
    Data_cis(i).query_p = linspace(0,1,1000);
    Data_cis(i).query_frame(1).A = Demos_cis(i).A(:,:,1);
    Data_cis(i).query_frame(1).b = Demos_cis(i).b(:,1);
    Data_cis(i).query_frame(2).A = Demos_cis(i).A(:,:,2);
    Data_cis(i).query_frame(2).b = Demos_cis(i).b(:,2);
    % Retrieve
    [Data_cis(i).expData_p, Data_cis(i).expSigma_p] = policy_cis_posi.GMR(Data_cis(i).query_p,Data_cis(i).query_frame);
end
%}
%{
figure;
for i = 1:M
    X = Demos_cis(i).data(2,:);
    Y = Demos_cis(i).data(3,:);
    Z = Demos_cis(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6],'LineWidth',1.2);
    hold on;
    X = Data_cis(i).expData_p(1,:);
    Y = Data_cis(i).expData_p(2,:);
    Z = Data_cis(i).expData_p(3,:);
    plot3(X,Y,Z,'b','LineWidth',1.8);
%     legend({'Demonstrated trajectories','Retrieved trajectories'});
end
grid on; axis equal;
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
view(128,20);
%}

% % Retrieve quaternion data
%{
for i = 1:M
    tmp_query_q = (Data_cis(i).query_frame(2).A(2:4,2:4))' * (Data_cis(i).expData_p - Data_cis(i).query_frame(2).b(2:4,1));
    Data_cis(i).query_q = tmp_query_q(3,:);
    Data_cis(i).qa = Demos_cis(i).qa;
    [Data_cis(i).expData_eta, Data_cis(i).expSigma_eta] = policy_cis_quat.GMR(Data_cis(i).query_q);
    Data_cis(i).expData_q = quatRegulate( policy_cis_quat.expmap(Data_cis(i).expData_eta) );
    Data_cis(i).expData_realq = quatRegulate( quatProduct(repmat(Data_cis(i).qa,[1,size(Data_cis(i).expData_q,2)]), Data_cis(i).expData_q) );
end
%}
%{
figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        [tmp_query_q, tmp_Index] = sort(Demos_cis(j).posi(3,:));
        tmp_q = quatRegulate(Demos_cis(j).q(:,tmp_Index));
        plot(tmp_query_q,tmp_q(i,:),'Color',[0.6,0.6,0.6]);
        hold on;
        [tmp_query_q, tmp_Index] = sort(Data_cis(j).query_q);
        tmp_q = quatRegulate(Data_cis(j).expData_q(:,tmp_Index));
        plot(tmp_query_q,tmp_q(i,:));
        hold on;
    end
    grid on;
end

figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        [tmp_query_q, tmp_Index] = sort(Data_cis(j).query_q);
        tmp_q = quatRegulate(Data_cis(j).expData_realq(:,tmp_Index));
        plot(tmp_query_q,tmp_q(i,:));
        hold on;
    end
    grid on;
end
%}

% % Retrieve wrench data
%{
for i = 1:M
    Data_cis(i).query_w = Data_cis(i).query_q;
    [Data_cis(i).expData_w, Data_cis(i).expSigma_w] = policy_cis_wren.GMR(Data_cis(i).query_w);
    Data_cis(i).pre_frame = fold2SE3(ret01dummy);
    Data_cis(i).cis_frame = fold2SE3(ret01final);
end
%}
%{
figure;
for i = 1:6
    subplot(6,1,i);
    for j = 1:M
        [tmp_query_w, tmp_Index] = sort(Demos_cis(j).posi(3,:));
        tmp_w = Demos_cis(j).ofk(:,tmp_Index);
        plot(tmp_query_w, tmp_w(i,:),'Color',[0.6,0.6,0.6]);
        hold on;
    end
    for j = 1:M
        [tmp_query_w, tmp_Index] = sort(Data_cis(j).query_w);
        tmp_w = Data_cis(j).expData_w(:,tmp_Index);
        plot(tmp_query_w, tmp_w(i,:));
        hold on;
    end
    grid on;
end
%}

%% Generalization dummy

% % Query p
%{
for i = M+1:M+MG
    Data_cis(i).query_p = linspace(0,1,1000);
    Data_cis(i).pre_frame = pq2SE3(Data_pre_w(i).expData_p(:,end)-[0.0;0.0;0.003], Data_pre_q(i).expData_realq(:,end));
    Data_cis(i).cis_frame = fold2SE3(ret01final);
    Data_cis(i).query_frame(1).A = eye(4); Data_cis(i).query_frame(1).A(2:4,2:4) = Data_cis(i).pre_frame(1:3,1:3);
    Data_cis(i).query_frame(2).A = eye(4); Data_cis(i).query_frame(2).A(2:4,2:4) = Data_cis(i).cis_frame(1:3,1:3);
    % Never forget the unit problem
    Data_cis(i).query_frame(1).b = zeros(4,1); Data_cis(i).query_frame(1).b(2:4,1) = Data_cis(i).pre_frame(1:3,4) * 1000;
    Data_cis(i).query_frame(2).b = zeros(4,1); Data_cis(i).query_frame(2).b(2:4,1) = Data_cis(i).cis_frame(1:3,4) * 1000;
    [Data_cis(i).expData_p, Data_cis(i).expSigma_p] = policy_cis_posi.GMR(Data_cis(i).query_p, Data_cis(i).query_frame);
end


% Data_cis(M+1).query_p = linspace(0,1,1000);
% Data_cis(M+1).pre_frame = fold2SE3(ret01dummy);
% Data_cis(M+1).cis_frame = fold2SE3(ret01final);
% Data_cis(M+1).query_frame(1).A = eye(4); Data_cis(M+1).query_frame(1).A(2:4,2:4) = Data_cis(M+1).pre_frame(1:3,1:3);
% Data_cis(M+1).query_frame(2).A = eye(4); Data_cis(M+1).query_frame(2).A(2:4,2:4) = Data_cis(M+1).cis_frame(1:3,1:3);
% % Never forget the unit problem
% Data_cis(M+1).query_frame(1).b = zeros(4,1); Data_cis(M+1).query_frame(1).b(2:4,1) = Data_cis(M+1).pre_frame(1:3,4) * 1000;
% Data_cis(M+1).query_frame(2).b = zeros(4,1); Data_cis(M+1).query_frame(2).b(2:4,1) = Data_cis(M+1).cis_frame(1:3,4) * 1000;
% [Data_cis(M+1).expData_p, Data_cis(M+1).expSigma_p] = policy_cis_posi.GMR(Data_cis(M+1).query_p, Data_cis(M+1).query_frame);
%}
%{
figure;
for i = 1:M
    X = Demos_cis(i).data(2,:);
    Y = Demos_cis(i).data(3,:);
    Z = Demos_cis(i).data(4,:);
    plot3(X,Y,Z,'Color',[0.6,0.6,0.6],'LineWidth',1.2);
    hold on;
end
for i = M+1 : M+MG
    X = Data_cis(M+1).expData_p(1,:);
    Y = Data_cis(M+1).expData_p(2,:);
    Z = Data_cis(M+1).expData_p(3,:);
    plot3(X,Y,Z,'r','LineWidth',1.8);
end
grid on; axis equal;
xlabel('x/mm');
ylabel('y/mm');
zlabel('z/mm');
view(128,20);
%}

% % Query q
%{
tmp_query_q = (Data_cis(M+1).query_frame(2).A(2:4,2:4))' * (Data_cis(M+1).expData_p - Data_cis(M+1).query_frame(2).b(2:4,1));
Data_cis(M+1).query_q = tmp_query_q(3,:);
Data_cis(M+1).qa = tform2quat(Data_cis(M+1).cis_frame)';
[Data_cis(M+1).expData_eta, Data_cis(M+1).expSigma_eta] = policy_cis_quat.GMR(Data_cis(M+1).query_q);
Data_cis(M+1).expData_q = quatRegulate( policy_cis_quat.expmap(Data_cis(M+1).expData_eta) );
Data_cis(M+1).expData_realq = quatRegulate(...
    quatProduct(repmat(Data_cis(M+1).qa,[1,size(Data_cis(M+1).expData_q,2)]), Data_cis(M+1).expData_q) );
%}
%{
figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        [tmp_query_q, tmp_Index] = sort(Demos_cis(j).posi(3,:));
        tmp_q = quatRegulate(Demos_cis(j).q(:,tmp_Index));
        plot(tmp_query_q,tmp_q(i,:),'Color',[0.6,0.6,0.6]);
        hold on;
    end
    [tmp_query_q, tmp_Index] = sort(Data_cis(M+1).query_q);
    tmp_q = quatRegulate(Data_cis(M+1).expData_q(:,tmp_Index));
    plot(tmp_query_q,tmp_q(i,:));
    grid on;
end

figure;
for i = 1:4
    subplot(4,1,i);
    for j = M+1:M+1
        [tmp_query_q, tmp_Index] = sort(Data_cis(j).query_q);
        tmp_q = quatRegulate(Data_cis(j).expData_realq(:,tmp_Index));
        plot(tmp_query_q,tmp_q(i,:));
        hold on;
    end
    grid on;
end
%}

% % Query w
%{
Data_cis(M+1).query_w = Data_cis(M+1).query_q;
[Data_cis(M+1).expData_w, Data_cis(M+1).expSigma_w] = policy_cis_wren.GMR(Data_cis(M+1).query_w);
%}
%{
figure;
for i = 1:6
    subplot(6,1,i);
    for j = M+1:M+1
        [tmp_query_w, tmp_Index] = sort(Data_cis(j).query_w);
        tmp_w = Data_cis(j).expData_w(:,tmp_Index);
        plot(tmp_query_w, tmp_w(i,:));
        hold on;
    end
    grid on;
end
%}
