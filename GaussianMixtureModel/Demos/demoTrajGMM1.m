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
Demos = cell(1,M);
for n=1:M
	Demos{n} = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),N)); %Resampling
end

%% Init. an ordinary traj-GMM

model = TrajGMMOne(5,2,3,1);    % K, DPos, DD, F
% model = model.initGMMTimeBased_TmpTime();

%% Load TP data

% For TP-Traj-GMM:
% load('Data\Data04_2d2frame.mat');
%   TPDemo struct:
%   |   data: D x N, demo data
%   |   A: D x D x F, orientation matrices
%   |   b: D x F, position vectors
%   |   TPData: D x F x N, demo data in each frame
