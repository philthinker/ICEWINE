%% demo_GMR01
% Gaussian mixture model (GMM) with time-based Gaussian mixture regression (GMR) 
% used for reproduction.

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 4; %Number of states in the GMM
model.nbVar = 3; %Number of variables [t,x1,x2]
model.dt = 0.001; %Time step duration
nbData = 200; %Length of each trajectory
nbSamples = 5; %Number of demonstrations

gmm = GMMZero(model.nbStates,model.nbVar,model.dt);

%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('Data/LetterC.mat');
% Data=[];
% for n=1:nbSamples
% 	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
% 	Data = [Data [[1:nbData]*model.dt; s(n).Data]]; 
% end

% nbData = size(demos{1}.pos,2);
dData = size(demos{1}.pos,1);
Data = zeros(dData+1,nbSamples*nbData);  % Note that the first row is time
Demos = cell(1,nbSamples);
for i = 1:nbSamples
    Data(2:end,(i-1)*nbData+1:i*nbData) = demos{i}.pos;
    Data(1,(i-1)*nbData+1:i*nbData) = (1:nbData)*gmm.dt;
    Demos{i} = [((1:nbData)*gmm.dt)',(demos{i}.pos)']; % Note that the first column is time
end

%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);
% model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
[DataOut, SigmaOut] = GMR(model, [1:nbData]*model.dt, 1, 2:model.nbVar); %see Eq. (17)-(19)

gmm = gmm.initGMMKMeans(Demos);
gmm = gmm.learnGMM(Demos);
[gmmDataOut, gmmSigmaOut] = gmm.GMR(((1:nbData)*model.dt)');

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,500]); 
%Plot GMM
subplot(1,2,1); hold on; axis off; title('GMM');
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.5 .5 .5]);
plotGMM(model.Mu(2:model.nbVar,:), model.Sigma(2:model.nbVar,2:model.nbVar,:), [.8 0 0], .5);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot GMR
subplot(1,2,2); hold on; axis off; title('GMR');
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.5 .5 .5]);
plotGMM(DataOut, SigmaOut, [0 .8 0], .03);
plot(DataOut(1,:),DataOut(2,:),'-','linewidth',2,'color',[0 .4 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%print('-dpng','graphs/demo_GMR01.png');
%pause;
%close all;

figure('position',[10,10,1300,500]); 
%Plot GMM
subplot(1,2,1); hold on; axis off; title('GMM');
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.5 .5 .5]);
% plotGMM((gmm.Mu(:,2:gmm.nVar))', gmm.Sigma(2:gmm.nVar,2:gmm.nVar,:), [.8 0 0], .5);
gmm.plotGMM2GMR(gmm.Mu(:,2:gmm.nVar), gmm.Sigma(2:gmm.nVar,2:gmm.nVar,:), [.8 0 0], .5);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot GMR
subplot(1,2,2); hold on; axis off; title('GMR');
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.5 .5 .5]);
% plotGMM(gmmDataOut', gmmSigmaOut, [0 .8 0], .03);
gmm.plotGMM2GMR(gmmDataOut, gmmSigmaOut, [0 .8 0], .03);
plot(DataOut(1,:),DataOut(2,:),'-','linewidth',2,'color',[0 .4 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
