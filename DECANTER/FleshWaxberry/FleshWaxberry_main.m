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
% Demos_lead_pre_xyzq = cell(1,M);
% Demos_lead_cis_xyzq = cell(1,M);
% for i = 1:M
%     Demos_lead_cis_xyzq{i} = SE3toPQ(fold2SE3( Demos_lead_cis{i} ));
%     Demos_lead_pre_xyzq{i} = SE3toPQ(fold2SE3( Demos_lead_pre{i} ));
% end
% 
% Demos_lead_pre_xyzq_dtw = iceDTW(Demos_lead_pre_xyzq,100);
% Demos_lead_cis_xyzq_dtw = iceDTW(Demos_lead_cis_xyzq,100);

% % Add demos to robots
% for i = 1:M
%     robot_pre = robot_pre.addCartesianDemo(Demos_lead_pre{i});
%     robot_cis = robot_cis.addCartesianDemo(Demos_lead_cis{i});
%     robot_pre_dtw = robot_pre_dtw.addCartesianDemoPQ(Demos_lead_pre_xyzq_dtw{i});
%     robot_cis_dtw = robot_cis_dtw.addCartesianDemoPQ(Demos_lead_cis_xyzq_dtw{i});
% end
% robot_pre = robot_pre.computeDemoQuaternion();
% robot_cis = robot_cis.computeDemoQuaternion();
% robot_pre_dtw = robot_pre_dtw.computeDemoCartesian();
% robot_cis_dtw = robot_cis_dtw.computeDemoCartesian();

% % Figure
%  % load('Data\lead0711.mat')
% dt = 0.02;
% robot_pre.plotCarteDemo();
% robot_pre.plotCarteDemoXYZ(dt);
% robot_pre.plotCarteDemoQuat(dt);
% robot_cis.plotCarteDemo();
% robot_cis.plotCarteDemoXYZ(dt);
% robot_cis.plotCarteDemoQuat(dt);
% robot_pre_dtw.plotCarteDemo();
% robot_pre_dtw.plotCarteDemoXYZ(dt);
% robot_pre_dtw.plotCarteDemoQuat(dt);
% robot_cis_dtw.plotCarteDemo();
% robot_cis_dtw.plotCarteDemoXYZ(dt);
% robot_cis_dtw.plotCarteDemoQuat(dt);

%% Policy I learning

% % TP-GMM 

%% Policy II learning

% % TP-GMM
