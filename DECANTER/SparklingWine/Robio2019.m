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

% % Draw the DTW diagram
% figure;
% for i = 1:7
%     subplot(7,1,i);
%     for j = 1:panda.NJointDemo
%         t = linspace(0,1,size(dj{j},1));
%         plot(t,dj{j}(:,i),'Color',[0.5,0.5,0.5]);
%         hold on;
%         t = linspace(0,1,size(panda.demoJoint{j},1));
%         plot(t,panda.demoJoint{j}(:,i),'Color',[0.63,0.13,0.94]);
%         ylabel(strcat('Joint ',int2str(i)));
%     end
%     grid on;
% end

%% Init panda by PandaZero

% panda = PandaZero(true);

% panda1 = PandaZero();
% panda2 = PandaZero();
% panda3 = PandaZero();

%% Add demonstration data to panda

% for i = 1:9
%     panda = panda.addJointDemo(demoJointDTW{i});
% %     panda = panda.addCartesianDemo(demoPose{i});
% end

%% Demonstration data show

% for i = 1:6
%     panda.plotJoint(i);
% end
% for i = 1:6
%     panda.plotCarte(i);
% end
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
gmmCorr = GMMRobio2019(gmm);

[gmmCorr,~] = gmmCorr.addJCorrection(correJ1Index, [demoJointPlus{1}(:,1), gmrJoint], correJ1);
gmrJoint5 = gmmCorr.plotComparison();
gmmCorr.plotGMMPerDimension(demoJointPlus,[1,0,0],0.5);
gmmCorr.plotAddedModel(demoJointPlus);
% totxt(gmrJoint5,5,4,'gmrJoint5');
% 
% gmmCorr = gmmCorr.addJCorrection(correJ2Index,[time,gmrJoint5],correJ2);
% gmrJoint6 = gmmCorr.plotComparison();
% gmmCorr.plotAddedModel(demoJointPlus);
% totxt(gmrJoint6,5,4,'gmrJoint6');

% % Plot the corrections in Cartesian space
% gmrPose = gmmCorrPanda.Panda.fkine(gmrJoint);
% gmrPose5 = gmmCorrPanda.Panda.fkine(gmrJoint5);
% gmrPose6 = gmmCorrPanda.Panda.fkine(gmrJoint6);
% gmmCorrPanda.plotGMRwithDemos({gmrPose(:,:,(280:838)),gmrPose5(:,:,(280:838))},gmrPose6(:,:,(280:838)));

% gmmCorr = gmmCorr.addJCorrection(correJ3Index,[time,gmrJoint6],correJ3);
% gmrJoint7 = gmmCorr.plotComparison();
% totxt(gmrJoint7,5,4,'gmrJoint7');

% panda.plotJointDemoPlus(dt,[demoJointPlus{1}(:,1),gmrJoint6]);
% gmmCorr.plotGMMPerDimension(demoJointPlus,[1,0,0],0.5);

%% Contrast

% % GMM + GMR

% [gmrJoint, gmrJointSigma] = gmm.GMR(demoJointPlus{1}(:,1));
% panda.plotJointDemoPlus(dt,[demoJointPlus{1}(:,1), gmrJoint]);
% totxt(gmrJoint,5,4,'gmrJoint3');

% % Plot GMR
% figure;
% gmrJointPlus = [demoJointPlus{1}(:,1),gmrJoint];
% for i = 1:7
%     subplot(7,1,i);
%     for j = 1:panda.NJointDemo
%         plot(demoJointPlus{j}(:,1),demoJointPlus{j}(:,i+1),'Color',[0.36,0.36,0.36]);
%         gmm.plotGMM2GMR(gmrJointPlus(:,[1,i+1]), gmrJointSigma([1,i+1],[1,i+1],:), [0 .8 0], .03);
%         hold on;
%     end
%     plot(gmrJointPlus(:,1),gmrJointPlus(:,i+1),'Color',[0,1,0]);  % It's green
%     grid on; ylabel(strcat('Joint ',int2str(i)));
%     axis([gmrJointPlus(1,1),gmrJointPlus(end,1),-inf,inf]);
% end

% % Plot3 GMR
% gmmCorrPanda = gmmCorr.addPanda(panda);
% dpfkine = cell(size(dj));
% for i = 1:size(dj,2)
%     dpfkine{i} = gmmCorrPanda.Panda.fkine(dj{i});
%     gmmCorrPanda.Panda = gmmCorrPanda.Panda.addCartesianDemo(dpfkine{i},true);
% end
% gmmCorrPanda.Panda.exeCartesian = gmmCorrPanda.Panda.fkine(gmrJoint);
% gmmCorrPanda.Panda.plotCarteDemo(true);
% gmmCorrPanda.plotGMRwithDemos(dpfkine,gmmCorrPanda.Panda.exeCartesian);
