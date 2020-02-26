%rcar2020

% Haopeng Hu
% 2020.02.24
% All rights reserved

%% load data
% Load data from the PlumWine project, or figure in 'Figure' folder
% load('Data\ur5kg0110.mat') for raw data;
% load('Data\ur5kgRCAR2020.mat') for simple data

%% TP-GMM for part I



%% DMP for part II


% % Six Ijspeert's DMPs
% tmpDMP = IjspeertDMP(6, 25,5,40, 0.001);
% dmps = repmat(tmpDMP,[1,6]);    % Indeed used dmps
% dmpsCon = dmps;                 % Used for contrast


% Learn one trajectory for test

% % Run an empty DMP
% [Y,x,fx] = tmpDMP.run(0,1,1);
% tmpDMP.plot(x,Y,1);
% tmpDMP.plotGaussian(x,fx,1);

% Learn a demo trajectory
% Construct dynamic data N x 3

% tmpData = demo(10).dataJ(:,3);
% tmpData = repmat(tmpData,[1,3]);
% tmpData(1,2) = 1e-6;
% tmpData(2:end,2) = (tmpData(2:end,2) - tmpData(1:end-1,1))/0.001 + 1e-6;
% tmpData(:,3) = tmpData(:,2);
% tmpData(2:end,3) = (tmpData(2:end,3) - tmpData(1:end-1,2))/0.001 + 1e-6;
% tmpDMP = tmpDMP.LWR(tmpData,1);

[Y,x,fx] = tmpDMP.run(0,1,1);
tmpDMP.plot(x,Y,1);
tmpDMP.plotGaussian(x,fx,1);
Y(99,:) = Y(end,:);
% tmpDMP.plotCompare(Y,tmpData,1);

%% Merge part I and part II


