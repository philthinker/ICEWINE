%demoTrajGMM0
%   Trajectory synthesis with a GMM with dynamic features (trajectory GMM).
%
%   Haopeng Hu
%   2019.10.24
%   All rights reserved

%% Data and GMM initialization

% load('Data\LetterS.mat');
nSample = 4;    % Num. of demos needed
nData = 100;    % Num. of data in one trajectory
gmm = TrajGMM(5,2,3,0.001,nSample,nData);   % nKernel, nVarPos, nDeriv, dt, nSample, nData
% % Resampling
Demos = cell(1,nSample);
for n=1:nSample
	Demos{n} = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nData)); %Resampling
end
[Data1,zeta1,x1] = gmm.dynamicDataGeneration(Demos);

%% Learn GMM param.

gmm = gmm.initGMMKMeans(Data1);
gmm = gmm.learnGMM(Data1);

%% Reproduction by trajectory-GMM


%% Figure