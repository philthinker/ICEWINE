% Robio2019

% Haopeng Hu
% 2019.09.25
% All rights reserved

%% Load demonstration data
% % If the data is raw, please run Robio2019Data in advance.

% load('fruits\robio2019.mat');

%% Temporal alignment

% demoJointDTW = iceDTW(demoJoint,100);

%% Init panda by PandaZero

% panda = PandaZero();

%% Add demonstration data to panda

% for i = 1:9
%     panda = panda.addJointDemo(demoJointDTW{i});
%     panda = panda.addCartesianDemo(demoPose{i});
% end

%% Demonstration data show

% for i = 1:6
%     panda.plotJoint(i);
% end
% for i = 1:6
%     panda.plotCarte(i);
% end
% panda.plotCarteDemo();
% panda.plotJointDemo();

%% GMM init

% % addpath('GaussianMixtureModel');
% gmm = GMMZero(10,8); dt = 0.001;    % Note that the very left column must be time series
% demoJointPlus = panda.demoJointChron(dt);
% gmm = gmm.initGMMKMeans(demoJointPlus);
% gmm = gmm.learnGMM(demoJointPlus);

%% GMM show

% panda.plotJointDemo();
% gmm = gmm.sortMu(1);
% gmm.plotGMMPerDimension(demoJointPlus,[1,0,0],0.5);

%% Find the critical positions

keys = gmm.Mu;  % Never forget that the first column is the time series
vars = gmm.Sigma;

% Get ride of the 

%% Contrast

% % GMM + GMR

% [gmrJoint, gmrJointSigma] = gmm.GMR(demoJointPlus{1}(:,1));
% panda.plotJointDemoPlus(dt,[demoJointPlus{1}(:,1), gmrJoint]);
% totxt(gmrJoint,5,4,'gmrJoint');
