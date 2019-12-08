%Sake_main

% Haopeng Hu
% 2019.12.07
% All rights reserved

% Well, it is a little regretful to miss the Rome trip.

%% Data init.

% % load('Data\20191204PandaData\20191204PandaData.mat');
% 
% % struct array data
% 
% demos.joint = joint1;
% demos.pose = pose1;
% demos = repmat(demos,[1,9]);
% 
% for i = 1:9
%     demos(i).joint = eval(strcat('joint',int2str(i)));
%     demos(i).pose = eval(strcat('pose',int2str(i)));
% end
% 
% % cell data
% 
% DemosJ = cell(1,9);
% DemosP = cell(1,9);
% 
% for i = 1:9
%     DemosJ{i} = eval(strcat('joint',int2str(i)));
%     DemosP{i} = eval(strcat('pose',int2str(i)));
% end

%% Robot init.

% panda = PandaZero(true);
% 
% % Add demos
% 
% for i =1:9
%     panda = panda.addJointDemo(DemosJ{i});
%     panda = panda.addCartesianDemo(DemosP{i});
% end
% 
% % Show demos
% 
% panda.plotJointDemo();
% panda.plotCarteDemo();

%% Aligner init.

%{

% Learn a GMM as the aligner

aligner = GMMSake(10,3);
aligner = aligner.initGMMKMeans(panda);
aligner = aligner.learnGMMEM(panda);

% Show the learned aligner

aligner.plotPandaDemos(panda);
plotGMM3SC(aligner.Mu,aligner.Sigma,[0.8,0,0],0.3);
view(3); axis equal;
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
grid on;

%}
