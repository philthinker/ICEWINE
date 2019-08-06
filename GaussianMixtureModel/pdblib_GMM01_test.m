%% demo_GMM01

%% Load Data and spline

% load('Data\LetterG.mat')  % demos
% Spline is not used here

nSamples = 6;
nData = size(demos{1}.pos,2);
dData = size(demos{1}.pos,1);
Data = zeros(dData,nSamples*nData);
iceData = cell(1,nSamples);
for i = 1:nSamples
    Data(:,(i-1)*nData+1:i*nData) = demos{i}.pos;
    iceData{i} = (demos{i}.pos)';
end

%% Init. GMM

nStates = 5;
nVar = 2;
GMMmodel = GMMZero(nStates,nVar);

model.nbStates = nStates;
model.nbVar = nVar;

%% Parameter Estimation

GMMmodel = GMMmodel.initGMMKMeans(iceData);
GMMmodel = GMMmodel.learnGMM(iceData);

%% Figure

figure('position',[10,10,700,500]); hold on; axis off;
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.5 .5 .5]);
GMMmodel.plotGMM2SC( [.8 0 0],.5);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

% print('-dpng','graphs/demo_GMM01.png');
% pause;
% close all;

