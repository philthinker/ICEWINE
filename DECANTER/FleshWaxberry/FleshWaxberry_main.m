%FleshWaxberry_main
%   PbD framework for assembly
%
%   Haopeng Hu
%   2020.07.11
%   All rights reserved


%% Demonstration

% load('Data\lead0711.mat')

% M = 9;

% % Init. robots
% robot_pre = PandaOne();
% robot_cis = PandaOne();
% robot_pre_dtw = PandaOne();
% robot_cis_dtw = PandaOne();

% % DTW (Both quat and xyz data are considered here)
%{
Demos_lead_pre_xyzq = cell(1,M);
Demos_lead_cis_xyzq = cell(1,M);
for i = 1:M
    Demos_lead_cis_xyzq{i} = downSample( SE3toPQ(fold2SE3( Demos_lead_cis{i} )),1);
    Demos_lead_pre_xyzq{i} = downSample( SE3toPQ(fold2SE3( Demos_lead_pre{i} )),2);
end

Demos_lead_pre_xyzq_dtw = iceDTW(Demos_lead_pre_xyzq,100);
Demos_lead_cis_xyzq_dtw = iceDTW(Demos_lead_cis_xyzq,100);

% Add demos to robots
for i = 1:M
%     robot_pre = robot_pre.addCartesianDemo(Demos_lead_pre{i});
%     robot_cis = robot_cis.addCartesianDemo(Demos_lead_cis{i});
    robot_pre_dtw = robot_pre_dtw.addCartesianDemoPQ(Demos_lead_pre_xyzq_dtw{i});
    robot_cis_dtw = robot_cis_dtw.addCartesianDemoPQ(Demos_lead_cis_xyzq_dtw{i});
end
% robot_pre = robot_pre.computeDemoQuaternion();
% robot_cis = robot_cis.computeDemoQuaternion();
robot_pre_dtw = robot_pre_dtw.computeDemoCartesian();
robot_cis_dtw = robot_cis_dtw.computeDemoCartesian();
%}

% % Figure
%  % load('Data\lead0711.mat')
%{
% dt = 0.02;
% robot_pre.plotCarteDemo();
% robot_pre.plotCarteDemoXYZ(dt);
% robot_pre.plotCarteDemoQuat(dt);
% robot_cis.plotCarteDemo();
% robot_cis.plotCarteDemoXYZ(dt);
% robot_cis.plotCarteDemoQuat(dt);
robot_pre_dtw.plotCarteDemo();
robot_pre_dtw.plotCarteDemoXYZ(dt);
robot_pre_dtw.plotCarteDemoQuat(dt);
robot_cis_dtw.plotCarteDemo();
robot_cis_dtw.plotCarteDemoXYZ(dt);
robot_cis_dtw.plotCarteDemoQuat(dt);
%}

%% Policy I learning

% % TP-GMM for position
% policy_pre = TPGMMOne(4,4,2);   % K D F

% % To TPDemo data
%{
tmpDemo = robot_pre_dtw.demoCartesian{1};

[tmpData,tmpA,tmpb] = extractTPCarte(tmpDemo);
Demos_tp_pre = policy_pre.TPDemoConstruct(tmpData,tmpA,tmpb,true);
Demos_tp_pre = repmat(Demos_tp_pre,[1,M]);
for i = 2:robot_pre_dtw.NCartesianDemo
    [tmpData,tmpA,tmpb] = extractTPCarte(robot_pre_dtw.demoCartesian{i});
    Demos_tp_pre(i) = policy_pre.TPDemoConstruct(tmpData,tmpA,tmpb,true);
end
%}

% % Learn TP-GMM for position
% policy_pre = policy_pre.initGMMTimeBased(Demos_tp_pre);
% policy_pre = policy_pre.learnGMM(Demos_tp_pre);

% % Retrieve position for the 1st demo to verify its effectivenss
% queryGMR_pre_posi = genDataStoration4TPGMR(linspace(0,1,200),Demos_tp_pre(1).A,Demos_tp_pre(1).b);
% [queryGMR_pre_posi.dataOut,queryGMR_pre_posi.Sigma] = policy_pre.GMR(queryGMR_pre_posi.dataIn,queryGMR_pre_posi.queryFrames);
%{
% robot_pre_dtw.plot3CarteDemoPlus((queryGMR.dataOut)');
figure;
hold on;
for i = 2:9
    tmpData = Demos_tp_pre(i).data;
    plot3(tmpData(2,:),tmpData(3,:),tmpData(4,:),'Color',[0.6,0.6,0.6]);
end
tmpData = Demos_tp_pre(1).data;
plot3(tmpData(2,:),tmpData(3,:),tmpData(4,:),'Color',[0,0,0.6]);
plot3(queryGMR_pre_posi.dataOut(1,:),queryGMR_pre_posi.dataOut(2,:),queryGMR_pre_posi.dataOut(3,:));
grid on; axis equal;
view(3);
%}

% % GMM in Riemannian manifold for quaternion
% We use the unit quaternion as in pre-assemby frame
% Only z position is considered for simplicity

% Init data
%{
Demos_tp_pre_quat = struct('A',repmat(eye(3),[1,1,2]),'data',[],'TPData',[]);
Demos_tp_pre_quat = repmat(Demos_tp_pre_quat,[1,M]); 
for i = 1:M
    tmpCarteDemo = robot_pre_dtw.demoCartesian{i};
    Demos_tp_pre_quat(i).data = rotm2quat( tmpCarteDemo(1:3,1:3,:) )';
    tmpN = size(tmpCarteDemo,3);
    Demos_tp_pre_quat(i).A(:,:,1) = tmpCarteDemo(1:3,1:3,1);
    Demos_tp_pre_quat(i).A(:,:,2) = tmpCarteDemo(1:3,1:3,end);
    Demos_tp_pre_quat(i).TPData = zeros(4,2,tmpN);
    for j = 1:tmpN
        for k = 1:2
            tmpA = Demos_tp_pre_quat(i).A(:,:,k);
            Demos_tp_pre_quat(i).TPData(:,k,j) = rotm2quat( tmpA' * tmpCarteDemo(1:3,1:3,j) )';
        end
    end
end
%}
% Figure to check
%{
figure;
ylabels = {'w','qx','qy','qz'};
for i = 1:M
    t = (1:size(Demos_tp_pre_quat(i).data,2))*0.02-0.02;
    for j = 1:4
        subplot(4,1,j);
        plot(t, permute( Demos_tp_pre_quat(i).TPData(j,2,:), [1,3,2]) );
        hold on; grid on;
        ylabel(ylabels{j});
    end
end
%}

% Init Riemannian GMM policy
% policy_pre_quat = RiemannianGMMZero(4,4); % K, D

% Reconstruct data struct
% Note that we init. and learn the GMM on the tangent space
% We ought to compute the data in tangent space first
%{
for i = 1:M
    Demos_tp_pre_quat(i).queryIn = permute( Demos_tp_pre(i).TPData(4,2,:), [1,3,2]);
    Demos_tp_pre_quat(i).dataMan = permute( Demos_tp_pre_quat(i).TPData(:,2,:), [1,3,2]);
    Demos_tp_pre_quat(i).dataTan = policy_pre_quat.logmap(Demos_tp_pre_quat(i).dataMan,policy_pre_quat.center);
    Demos_tp_pre_quat(i).RiemannianData = policy_pre_quat.constructRiemannianData(...
        Demos_tp_pre_quat(i).dataMan,...
        Demos_tp_pre_quat(i).dataTan,...
        Demos_tp_pre_quat(i).queryIn);
end
%}
% Init. and learn the GMM
%{
N = size(Demos_tp_pre_quat(1).dataTan,2);
tmpDemosData = demoAugment(Demos_tp_pre_quat,{'queryIn','dataTan'},N);
policy_pre_quat = policy_pre_quat.initGMMKBins(tmpDemosData, M,N );
tmpRiemannianData = policy_pre_quat.constructRiemannianData(...
    demoAugment(Demos_tp_pre_quat,{'dataMan'},N),...
    demoAugment(Demos_tp_pre_quat,{'dataTan'},N),...
    demoAugment(Demos_tp_pre_quat,{'queryIn'},N));
policy_pre_quat = policy_pre_quat.learnGMM(tmpRiemannianData);
%}

% Retrieve the demos to check
% [tmpRiemannianData.expDataM,tmpRiemannianData.expData,tmpRiemannianData.expSigma] =...
%     policy_pre_quat.GMR(tmpRiemannianData.queryIn);
%{
figure;
tmpRiemannianData_pre = tmpRiemannianData;
tmpRiemannianData_pre.expDataM = policy_pre_quat.quatRegulate(tmpRiemannianData.expDataM);
N = size(Demos_tp_pre_quat(1).dataMan,2);
t = (1:N)*dt-dt; ylabels = {'qw','qx','qy','qz'};
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        plot(t,Demos_tp_pre_quat(j).dataMan(i,:),'Color',[0.6,0.6,0.6]);
        hold on;
    end
    plot(t,tmpRiemannianData_pre.expDataM(i,1:N),'Color',[0.6,0.6,0]);
    grid on;
    ylabel(ylabels{i});
end
%}

%% Policy II learning

% % TP-GMM for position

% % Figure the origin demos for comparison
% robot_cis_dtw.plotCarteDemo();
% robot_cis_dtw.plotCarteDemoXYZ(0.01);
% policy_cis = TPGMMOne(6,4,2);

% % To TPDemo data
%{
tmpDemo = robot_cis_dtw.demoCartesian{1};
[tmpData,tmpA,tmpb] = extractTPCarte(tmpDemo);
Demos_tp_cis = policy_cis.TPDemoConstruct(tmpData,tmpA,tmpb,true);
Demos_tp_cis = repmat(Demos_tp_cis,[1,M]);
for i = 2:robot_pre_dtw.NCartesianDemo
    [tmpData,tmpA,tmpb] = extractTPCarte(robot_cis_dtw.demoCartesian{i});
    Demos_tp_cis(i) = policy_cis.TPDemoConstruct(tmpData,tmpA,tmpb,true);
end
%}

% % Scale the data to avoid computational issue
%{
tmpDemo = robot_cis_dtw.demoCartesian{1};
tmpDemo(1:3,4,:) = tmpDemo(1:3,4,:)*1000;   % Unit transformation (mm here)
[tmpData,tmpA,tmpb] = extractTPCarte(tmpDemo);
Demos_tp_cis_mm = policy_cis.TPDemoConstruct(tmpData,tmpA,tmpb,true);
Demos_tp_cis_mm = repmat(Demos_tp_cis_mm,[1,M]);
for i = 2:robot_pre_dtw.NCartesianDemo
    tmpDemo = robot_cis_dtw.demoCartesian{i};
    tmpDemo(1:3,4,:) = tmpDemo(1:3,4,:)*1000;
    [tmpData,tmpA,tmpb] = extractTPCarte(tmpDemo);
    Demos_tp_cis_mm(i) = policy_cis.TPDemoConstruct(tmpData,tmpA,tmpb,true);
end
%}

% % Learn the policy
% policy_cis = policy_cis.initGMMTimeBased(Demos_tp_cis_mm);
% policy_cis = policy_cis.learnGMM(Demos_tp_cis_mm);

% % Retrieve the 1st demo
%{
queryGMR_cis_posi = genDataStoration4TPGMR(linspace(0,1,200),Demos_tp_cis_mm(2).A,Demos_tp_cis_mm(2).b);
[queryGMR_cis_posi.dataOut,queryGMR_cis_posi.Sigma] = policy_cis.GMR(queryGMR_cis_posi.dataIn,queryGMR_cis_posi.queryFrames);
% % Figure to check
figure;
hold on;
for i = 1:9
    tmpData = Demos_tp_cis(i).data * 1000;
    plot3(tmpData(2,:),tmpData(3,:),tmpData(4,:),'Color',[0.6,0.6,0.6]);
end
tmpData = Demos_tp_cis(2).data * 1000;
plot3(tmpData(2,:),tmpData(3,:),tmpData(4,:),'Color',[0,0,0.6]);
plot3(queryGMR_cis_posi.dataOut(1,:),queryGMR_cis_posi.dataOut(2,:),queryGMR_cis_posi.dataOut(3,:),'Color',[0.6,0.6,0]);
grid on; axis equal;
view(3);
%}

% % GMM in Riemannian manifold for quaternion
%{
Demos_tp_cis_quat = struct('A',repmat(eye(3),[1,1,2]),'data',[],'TPData',[]);
Demos_tp_cis_quat = repmat(Demos_tp_cis_quat,[1,M]); 
for i = 1:M
    tmpCarteDemo = robot_cis_dtw.demoCartesian{i};
    Demos_tp_cis_quat(i).data = rotm2quat( tmpCarteDemo(1:3,1:3,:) )';
    tmpN = size(tmpCarteDemo,3);
    Demos_tp_cis_quat(i).A(:,:,1) = tmpCarteDemo(1:3,1:3,1);
    Demos_tp_cis_quat(i).A(:,:,2) = tmpCarteDemo(1:3,1:3,end);
    Demos_tp_cis_quat(i).TPData = zeros(4,2,tmpN);
    for j = 1:tmpN
        for k = 1:2
            tmpA = Demos_tp_cis_quat(i).A(:,:,k);
            Demos_tp_cis_quat(i).TPData(:,k,j) = rotm2quat( tmpA' * tmpCarteDemo(1:3,1:3,j) )';
        end
    end
end

figure;
ylabels = {'w','qx','qy','qz'};
for i = 1:M
    t = (1:size(Demos_tp_cis_quat(i).data,2))*0.02-0.02;
    for j = 1:4
        subplot(4,1,j);
        plot(t, permute( Demos_tp_cis_quat(i).TPData(j,2,:), [1,3,2]) );
        hold on; grid on;
        ylabel(ylabels{j});
    end
end
%}

% Init Riemannian GMM policy
% policy_cis_quat = RiemannianGMMZero(4,4); % K, D

% Reconstruct data struct
% Note that we init. and learn the GMM on the tangent space
% We ought to compute the data in tangent space first
%{
for i = 1:M
    Demos_tp_cis_quat(i).queryIn = permute( Demos_tp_cis(i).TPData(4,2,:), [1,3,2]);
    Demos_tp_cis_quat(i).dataMan = permute( Demos_tp_cis_quat(i).TPData(:,2,:), [1,3,2]);
    Demos_tp_cis_quat(i).dataTan = policy_cis_quat.logmap(Demos_tp_cis_quat(i).dataMan,policy_cis_quat.center);
    Demos_tp_cis_quat(i).RiemannianData = policy_cis_quat.constructRiemannianData(...
        Demos_tp_cis_quat(i).dataMan,...
        Demos_tp_cis_quat(i).dataTan,...
        Demos_tp_cis_quat(i).queryIn);
end
%}

% Init. and learn the GMM
%{
N = size(Demos_tp_cis_quat(1).dataTan,2);
tmpDemosData = demoAugment(Demos_tp_cis_quat,{'queryIn','dataTan'},N);
policy_cis_quat = policy_cis_quat.initGMMKBins(tmpDemosData, M,N );
tmpRiemannianData = policy_cis_quat.constructRiemannianData(...
    demoAugment(Demos_tp_cis_quat,{'dataMan'},N),...
    demoAugment(Demos_tp_cis_quat,{'dataTan'},N),...
    demoAugment(Demos_tp_cis_quat,{'queryIn'},N));
policy_cis_quat = policy_cis_quat.learnGMM(tmpRiemannianData);
%}


