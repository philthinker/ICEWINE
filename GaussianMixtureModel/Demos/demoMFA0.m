%demoMFA0
% Mixture of factor analysers (MFA).
%
%   Haopeng Hu
%   2021.01.07
%   All rights reserved

addpath('./CELLAR/pbdlib-matlab-master/demos/m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 4; %Number of states in the GMM
model.nbVar = 4; %Number of variables [x1,x2,x3,x4]
model.nbFA = 1; %Dimension of the subspace (number of factor analyzers)
nbData = 200; %Length of each trajectory
nbSamples = 5; %Number of demonstrations


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('./CELLAR/pbdlib-matlab-master/demos/data/2Dletters/C.mat'); %Load x1,x2 variables
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
end
demos=[];
load('./CELLAR/pbdlib-matlab-master/demos/data/2Dletters/D.mat'); %Load x3,x4 variables
Data=[];
for n=1:nbSamples
	s(n).Data = [s(n).Data; spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData))]; %Resampling
	Data = [Data s(n).Data]; 
end


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);
model0 = EM_GMM(Data, model); %for comparison
model = EM_MFA(Data, model); 

gaussPDF(Data(:,1),model.Mu(:,1),model.Sigma(:,:,1))

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,500]); 
for i=1:2
	subplot(1,2,i); hold on; box on; 
	plot(Data((i-1)*2+1,:),Data(i*2,:),'.','markersize',8,'color',[.7 .7 .7]);
	plotGMM(model0.Mu((i-1)*2+1:i*2,:), model0.Sigma((i-1)*2+1:i*2,(i-1)*2+1:i*2,:), [.8 .8 .8], .5);
	plotGMM(model.Mu((i-1)*2+1:i*2,:), model.Sigma((i-1)*2+1:i*2,(i-1)*2+1:i*2,:), [.8 0 0], .5);
	axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
	xlabel(['x_' num2str((i-1)*2+1)]); ylabel(['x_' num2str(i*2)]);
end

