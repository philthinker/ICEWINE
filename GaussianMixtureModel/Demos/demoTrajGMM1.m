%demoTrajGMM1   A demo for class @TrajGMMOne
%
% Haopeng Hu
% 2020.03.07
% All rights reserved
%

%% Load hand-writting data

% For ordinary Traj-GMM:
% load('Data\LetterS.mat');
M = 4;    % Num. of demos needed
N = 100;    % Num. of data in one trajectory
Demos = []; Demos.data = []; Demos = repmat(Demos,[1,M]);
for n=1:M
	Demos(n).data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),N)); %Resampling
end
dt = 0.001;

%% Init. an ordinary traj-GMM

model = TrajGMMOne(5,2,3,1);    % K, DPos, DD, F
model = model.initGMMTimeBased_TmpTime(Demos,dt);
% model = model.initGMMKMeans(Demos,dt);
model = model.learnGMM(Demos,dt);

%% Reproduction of the ordinary traj-GMM

query = model.deriveQuery4Reprod(Demos(1),100);
[expData,expSigma] = model.reproduct(query,dt);

% Figure
figure;hold on;
% Plot expected data & covariances
	for n=1:1 %nbSamples
		plotGMM2SC(expData([1,2],:), expSigma([1,2],[1,2],:), [1 .2 .2],.2);
	end
% Plot learned GMM
plotGMM2SC(model.Mu([1,2],:), model.Sigma([1,2],[1,2],:), [.5 .5 .5],.8);
for n=1:M % Note that we have amplify the data for numerical computation
    plot(Demos(n).data(1,:)*1e2,Demos(n).data(2,:)*1e2,'-','lineWidth',1,'color',[.2 .2 .2]);hold on;
end
plot(expData(1,:), expData(2,:), '-','lineWidth',2.5,'color',[.8 0 0]);

axis equal; axis square;
xlabel(('$x_1$'),'interpreter','latex','fontsize',18);
ylabel(('$x_2$'),'interpreter','latex','fontsize',18);

%% Load TP data

% For TP-Traj-GMM:
% load('Data\Data04_2d2frame.mat');
%   TPDemo struct:
%   |   data: D x N, demo data
%   |   A: D x D x F, orientation matrices
%   |   b: D x F, position vectors
%   |   TPData: D x F x N, demo data in each frame
