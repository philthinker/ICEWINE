%demoGMR1 A simple demo for class @GMMOne
%
% Haopeng Hu
% 2020.03.05
% All rights reserved
%

%% Data

% load('Data\LetterG.mat');
nSamples = 10;
dt = 0.01;
nData = size(demos{1}.pos,2);   % Only position considered
dData = size(demos{1}.pos,1);
Data = zeros(dData,nSamples*nData);
Demos = cell(1,nSamples);
for i = 1:nSamples
    Data(:,(i-1)*nData+1:i*nData) = demos{i}.pos;   % Not used in Parameter estimation but in Figure
    Demos{i} = [(1:nData).*dt;(demos{i}.pos)];    % Directly used in Parameter estimation, the 1st column is time
end

%% Init. a GMM

gmm = GMMOne(6,3);  % Note that the 1st var. is time sequence

%% Learn the GMM

gmm = gmm.initGMMTimeBased(gmm.dataRegulate(Demos));
gmm = gmm.learnGMM(gmm.dataRegulate(Demos));

%% Reproduct

[expData,expSigma] = gmm.GMR((1:nData).*dt);

%% Figure

figure; 
%Plot GMM
subplot(1,2,1); hold on; axis off; title('GMM');
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.5 .5 .5]);
plotGMM2SC(gmm.Mu(2:3,:),gmm.Sigma(2:3,2:3,:),[1,0,0],0.5);
axis equal;
%Plot GMR
subplot(1,2,2); hold on; axis off; title('GMR');
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.5 .5 .5]);
plotGMM2SC(expData,expSigma,[0,1,0],0.3);
axis equal;


