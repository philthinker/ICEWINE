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

%{ 
% % Init. a pure PandaZero object

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

%}

% panda = PandaSake();
% for i = 1:length(DemosJ)
%     panda = panda.addJointDemo(DemosJ{i});
%     panda = panda.addCartesianDemo(DemosP{i});
% end
% panda = panda.initDemoFK();
% panda.plotJointDemo();
% panda.plotCarteDemo();

%% Try GMM

%{
% % GMM without time variable
% % Failed! You cannot ignore the influence to time variable.

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

%{
% Try DTW in joint space and GMM + GMR in Cartesian space
%
% GMMPandaDTW1219-10g.fig (10 GMs)
% GMMPandaDTW1219-15g.fig (15 GMs)
% GMMPandaDTW1219-15g-GMM.fig (15 GMs together with GMM)
% panda1204demo1219-15g,mat

panda.plotCarteDemo();
panda.plotJointDemo();

DemosJdtw = iceDTW(DemosJ,100);
pandaDTW = PandaSake();
for i = 1:length(DemosJdtw)
    pandaDTW = pandaDTW.addJointDemo(DemosJdtw{i});
    pandaDTW = pandaDTW.addCartesianDemo(DemosP{i});    % It does not matter
end
pandaDTW = pandaDTW.initDemoFK();   % Time consuming

pandaDTW.plotJoint();
pandaDTW.plotCarte();

gmm = GMMOne(15,4); % t,x,y,z

% Init. GMM w.r.t. time
Data = pandaDTW.get_demoFK_position(1e-3);
gmm = gmm.initGMMTimeBased(Data);

gmm = gmm.learnGMM(Data);

[expData,expSigma] = gmm.GMR((0:730)*1e-3);

pandaDTW = pandaDTW.set_exeCartesian(expData);

pandaDTW.plotCarteDemo(true);
% plotGMM3SC(gmm.Mu(2:end,:),gmm.Sigma(2:end,2:end,:),[0.8,0,0],0.3);   % Time consuming
%}

%{
% Try DTW in joint space and GMM + GMR in joint space
% Note that there are too much data and curse of dimension
% Nothing better than GMM + GMR in Cartesian space
% GMMPandaDTW1219-15g-J.fig (15 GMs)
% GMMPandaDTW1219-15g-J-joint.fig (15 GMs)
% GMMPandaDTW1219-15g-micro-J.fig (15 GMs together with GMM)
% panda1204demo1219-15g-J.mat

gmm = GMMOne(15,8); % t, q1 to q7

[demos,pandaDTWJ] = pandaDTW.sparse_demoFK();
Data = pandaDTWJ.get_demoFK_joint(1e-3);

gmm = gmm.initGMMTimeBased(Data);
gmm = gmm.learnGMM(Data);

[expData,expSigma] = gmm.GMR((0:365-1)*1e-3);
expDataPose = pandaDTWJ.fkine(expData');
pandaDTWJ = pandaDTWJ.set_exeCartesian(expDataPose);

pandaDTWJ.plotCarteDemo(true);

pandaDTWJ = pandaDTWJ.set_exeJoint(expData);
pandaDTWJ.plotJointDemo(true);
%}

%% Try GPR

%{
% GP by position (No DTW)
% Totally failed!
% panda1204demo1224-gp.mat

Data = panda.get_demoFK_position(1e-3); % Note that large amount of data leads to very slow computation
Data = Data(:,(1:3:size(Data,2)));

gp = GPZero(Data);
gp = gp.setParam(1e0,1e1,1e-2);
gp = gp.preGPR();
expData = gp.GPR(linspace(0,max(Data(1,:)),500));

panda = panda.set_exeCartesian(expData.Data(2:4,:));
panda.plotCarteDemo(true);
%}

%{
% GP by position (DTW in joint space)
% Totally failed!
% panda1204demo1224-gp.mat

Data = pandaDTW.get_demoFK_position(1e-3);
Data = Data(:,(1:3:size(Data,2)));

gp = GPZero(Data);
gp = gp.setParam(1e0,1e1);
gp = gp.preGPR();
expData = gp.GPR(linspace(0,max(Data(1,:)),500));

pandaDTW = pandaDTW.set_exeCartesian(expData.Data(2:4,:));
pandaDTW.plotCarteDemo(true);
%}

%{
% GP by joint (DTW in joint space)
% Partially succeeded. But the problem has not been solved totally
% GPRPandaDTW1224Joint-J.fig
% GPRPandaDTW1224Joint-P.fig
% panda1204demo1224-gp.mat

Data = pandaDTWJ.get_demoFK_joint(1e-3);
Data = Data(:,(1:2:size(Data,2)));

gp = GPZero(Data);
gp = gp.setParam(1e0,1e1);
gp = gp.preGPR();
expData = gp.GPR(linspace(0,max(Data(1,:)),500));

pandaDTWJ = pandaDTWJ.set_exeJoint(expData.Data(2:8,:));
pandaDTWJ.plotCarteDemo(true);
pandaDTWJ.plotJointDemo(true);
%}

%% Try SEDS


