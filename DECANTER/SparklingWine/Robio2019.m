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

% keys = gmm.Mu;  % Never forget that the first column is the time series
% vars = zeros(size(keys));   

% % Get ride of the keys whose vars are large in each dimensiton
% tmpIndex = ones(size(keys,1),1);
% for i = 1:size(keys,1)
%     vars(i,:) = 2*sqrt((diag(gmm.Sigma(:,:,i)))');
%     tmpMean = mean(vars(i,[2,3,5,6,7,8]));
%     if tmpMean >= 0.2
%         tmpIndex(i) = 0;
%     end
% end
% tmpMean = zeros(panda.NJointDemo,7);    tmpMean2 = tmpMean;
% for i = 1:panda.NJointDemo
%     tmpMean(i,:) = panda.demoJoint{i}(1,:);
%     tmpMean2(i,:) = panda.demoJoint{i}(end,:);
% end
% keys = [[0,mean(tmpMean,1)]; keys(tmpIndex==1,:); [1,mean(tmpMean2,1)]];
% keys = keys(tmpIndex == 1,:);
% clear tmpMean tmpMean2 tmpIndex i

%% Motion Plan

% % Peter Corke's robotics toolbox
% exeJoint = panda.jtraj(keys(:,2:8),24); % Never forget the time series in the first column
% panda.exeJoint = exeJoint;
% panda.plotJoint(0);
% totxt(exeJoint,5,4,'exeJoint1');

% Sparse the GMR results
exeJoint = panda.jSparse(demoJoint0929,0.0025);
panda.exeJoint = exeJoint;
panda.plotJoint(0);
totxt(exeJoint,5,4,'exeJoint2');

%% COACH



%% Contrast

% % GMM + GMR

% [gmrJoint, gmrJointSigma] = gmm.GMR(demoJointPlus{1}(:,1));
% panda.plotJointDemoPlus(dt,[demoJointPlus{1}(:,1), gmrJoint]);
% totxt(gmrJoint,5,4,'gmrJoint');
