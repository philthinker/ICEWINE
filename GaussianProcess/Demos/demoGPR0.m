%demoGPR0 A demo for GPZero class

% Haopeng Hu
% 2019.12.19
% All rights reserved

%% Data

% load('Data\LetterG.mat');

% Note that too many points lead to very slow computation
nData = 20;    % No more than nData
M = 1;
for n=1:M
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nData)); %Resampling
	tt = [1:nData/2,3*nData/4:nData];  %Simulate missing data
	s(n).Data = [tt; s(n).Data(:,tt)];
	Data = [Data s(n).Data]; 
end

%% GP init.

gp = GPZero(Data);
gp = gp.setParam(1e0,1e1,1e-2);
gp = gp.preGPR();

%% GPR

expData = gp.GPR(linspace(1,nData,100));

%% Figure

figure; hold on;
plotGMM(expData.Data(2:3,:),expData.Sigma*1E1,[1 .2 .2],.2);
plot(expData.Data(2,:), expData.Data(3,:), '-','lineWidth',3.5,'color',[.8 0 0]);
grid on; axis equal; axis square;
