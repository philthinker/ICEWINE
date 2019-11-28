%demoHMM0
%   Hidden Markov Model demo

%   Haopeng Hu
%   2019.11.27
%   All rights reserved

%% Data initialization
% % Once you have run the following codes, it is recommended to comment them.

% % load('Data/letterG.mat');
% nSamples = 5;
% nData = 50;
% Data=[];    % Used in Figure section
% Demos = cell(1,nSamples);
% for n=1:nSamples
% 	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nData)); %Resampling
% 	s(n).nData = size(s(n).Data,2);
%     Demos{n} = s(n).Data;
% 	Data = [Data s(n).Data]; 
% end

%% HMM initialization

% hmm = HMMZero(5,2);
% hmm = hmm.initHMMKMeans(Demos);
% hmm = hmm.initTrans('l',nData); % 'r','u' or ('l',N)

%% Learn HMM by EM algorithm

% hmm = hmm.learnHMM(Demos);

%% Figure

figure('position',[10,10,1300,600],'color',[1 1 1]);
clrmap = lines(model.nbStates);
%Plot spatial data
subplot(1,2,1); axis off; hold on; 
plot(Data(1,:), Data(2,:), '.', 'linewidth', 2, 'color', [.3 .3 .3]);
for i=1:model.nbStates
	plotGMM(model.Mu(:,i), model.Sigma(:,:,i), clrmap(i,:), .6);
end
axis equal;
%Plot transition information
subplot(1,2,2); axis off; hold on; 
plotHMM(model.Trans, model.StatesPriors);
axis([-1 1 -1 1]*1.9); 
