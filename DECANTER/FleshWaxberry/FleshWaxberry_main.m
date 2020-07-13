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

%% Policy I Retrieving



%% Policy II Retrieving



%% Policy I Generalization


