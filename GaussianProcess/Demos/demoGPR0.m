%demoGPR0 A demo for GPZero class

% Haopeng Hu
% 2019.12.19
% All rights reserved

%% Data

N = 20;
load('GaussianProcess\Demos\Data\LetterG.mat');

%% GP init.

gp = GPZero(Data);
gp = gp.setParam(1e0,1e1,1e-2);
gp = gp.preGPR();

%% GPR

expData = gp.GPR(linspace(1,N,100));

%% Figure

figure; hold on;
plotGMM2SC(expData.Data(2:3,:),expData.Sigma*1E1,[1 .2 .2],.2);
plot(expData.Data(2,:), expData.Data(3,:), '-','lineWidth',3.5,'color',[.8 0 0]);
grid on; axis equal; axis square;
