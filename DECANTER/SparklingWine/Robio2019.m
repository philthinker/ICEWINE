% Robio2019

% Haopeng Hu
% 2019.09.25
% All rights reserved

%% Init. demonstration data

% dj = {dj1,dj2,dj3,dj4,dj5,dj6,dj7,dj8,dj9};
% for i = 1:9
%     dj{i}(:,3) = dj{i}(:,3)*0;
% end

% dj1 = dj(1:3);
% dj2 = dj(4:6);
% dj3 = dj(7:9);

%% Init. correction data



%% Load demonstration data
% % If the data is raw, please run Robio2019Data in advance.

% load('fruits\robio2019-5.mat');

%% Temporal alignment

% demoJointDTW = iceDTW(dj,100);

% demoJointDTW1 = iceDTW(dj1,100);
% demoJointDTW2 = iceDTW(dj2,100);
% demoJointDTW3 = iceDTW(dj3,100);

%% Init panda by PandaZero

% panda = PandaZero();

% panda1 = PandaZero();
% panda2 = PandaZero();
% panda3 = PandaZero();

%% Add demonstration data to panda

% for i = 1:9
%     panda = panda.addJointDemo(demoJointDTW{i});
%     panda = panda.addCartesianDemo(demoPose{i});
% end

% for i = 1:3
%     panda1 = panda1.addJointDemo(demoJointDTW1{i});
%     panda2 = panda2.addJointDemo(demoJointDTW2{i});
%     panda3 = panda3.addJointDemo(demoJointDTW3{i});
% end
% panda1.plotJointDemo();
% panda2.plotJointDemo();
% panda3.plotJointDemo();

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

% % dt = 0.001;
% % gmm1 = GMMZero(10,8);
% % demoJointPlus1 = panda1.demoJointChron(dt);
% % gmm1 = gmm1.initGMMKMeans(demoJointPlus1);
% % gmm1 = gmm1.learnGMM(demoJointPlus1);
% % gmm1 = gmm1.sortMu(1);
% % gmm1.plotGMMPerDimension(demoJointPlus1,[1,0,0],0.5);
% % 
% % gmm2 = GMMZero(10,8);
% % demoJointPlus2 = panda2.demoJointChron(dt);
% % gmm2 = gmm2.initGMMKMeans(demoJointPlus2);
% % gmm2 = gmm2.learnGMM(demoJointPlus2);
% % gmm2 = gmm2.sortMu(1);
% % gmm2.plotGMMPerDimension(demoJointPlus2,[1,0,0],0.5);
% % 
% % gmm3 = GMMZero(10,8);
% % demoJointPlus3 = panda3.demoJointChron(dt);
% % gmm3 = gmm3.initGMMKMeans(demoJointPlus3);
% % gmm3 = gmm3.learnGMM(demoJointPlus3);
% % gmm3 = gmm3.sortMu(1);
% % gmm3.plotGMMPerDimension(demoJointPlus3,[1,0,0],0.5);

%% GMM show

% panda.plotJointDemo();
% gmm = gmm.sortMu(1);
% gmm.plotGMMPerDimension(demoJointPlus,[1,0,0],0.5);

%% Find the critical positions

% keys = gmm.Mu;  % Never forget that the first column is the time series
% vars = zeros(size(keys));   
% 
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

% % Sparse the GMR results
% exeJoint = panda.jSparse(gmrJoint,0.05);
% panda.exeJoint = exeJoint;
% panda.plotJoint(0);
% totxt(exeJoint,5,4,'gmrJoint3');

%% COACH

% time = demoJointPlus{1}(:,1);
% gmmCorr = GMMRobio2019(gmm);
% 
% [gmmCorr,~] = gmmCorr.addJCorrection(correJ1Index, [demoJointPlus{1}(:,1), gmrJoint], correJ1);
% gmrJoint5 = gmmCorr.plotComparison();
% totxt(gmrJoint5,5,4,'gmrJoint5');
% 
% gmmCorr = gmmCorr.addJCorrection(correJ2Index,[time,gmrJoint5],correJ2);
% gmrJoint6 = gmmCorr.plotComparison();
% totxt(gmrJoint6,5,4,'gmrJoint6');

gmmCorr = gmmCorr.addJCorrection(correJ3Index,[time,gmrJoint6],correJ3);
gmrJoint7 = gmmCorr.plotComparison();
totxt(gmrJoint7,5,4,'gmrJoint7');

% panda.plotJointDemoPlus(dt,[demoJointPlus{1}(:,1),gmrJoint6]);
gmmCorr.plotGMMPerDimension(demoJointPlus,[1,0,0],0.5);

%% Contrast

% % GMM + GMR

% [gmrJoint, gmrJointSigma] = gmm.GMR(demoJointPlus{1}(:,1));
% panda.plotJointDemoPlus(dt,[demoJointPlus{1}(:,1), gmrJoint]);
% totxt(gmrJoint,5,4,'gmrJoint3');

% % gmrJoint1 = gmm1.GMR(demoJointPlus1{1}(:,1));
% % panda1.plotJointDemoPlus(dt,[demoJointPlus1{1}(:,1), gmrJoint1]);
% % totxt(gmrJoint1,5,4,'gmrJoint4-1');
% % 
% % gmrJoint2 = gmm1.GMR(demoJointPlus2{1}(:,1));
% % panda2.plotJointDemoPlus(dt,[demoJointPlus2{1}(:,1), gmrJoint2]);
% % totxt(gmrJoint2,5,4,'gmrJoint4-2');
% % 
% % gmrJoint3 = gmm1.GMR(demoJointPlus3{1}(:,1));
% % panda3.plotJointDemoPlus(dt,[demoJointPlus3{1}(:,1), gmrJoint3]);
% % totxt(gmrJoint3,5,4,'gmrJoint4-3');
