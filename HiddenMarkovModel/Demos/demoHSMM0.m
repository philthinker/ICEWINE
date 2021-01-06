%demoHMM0
%   Hidden semi-Markov Model demo
%   Model learning and state sequence reconstruction.
%
%   Haopeng Hu
%   2019.11.27
%   All rights reserved
%
% Variable duration model implemented as a hidden semi-Markov model 
% (simplified version by encoding the state duration after EM).

%% Initialization

M = 10;
N = 100;
hsmm = HSMMZero(2,5);

%% Load handwriting data

load('./HiddenMarkovModel/Demos/Data/letterVPosi.mat');

%% Learning

hsmm = hsmm.initHMMKbins(Demos);
hsmm = hsmm.initTransLeftRight(N);
hsmm = hsmm.leanHMM(Demos);

%% Reconstruction of states probability sequence

[ht, seq] = hsmm.reconstructStSeq_StandardFW(N);

%% Figures

figure;
%Spatial plot of the data
axis off; hold on; 
for i = 1:M
    Data = Demos{i};
    plot(Data(1,:), Data(2,:), '.', 'color', [.3 .3 .3]);
end
for i=1:hsmm.K
	plotGMM2SC(hsmm.Mu(:,i), hsmm.Sigma(:,:,i), Morandi_carnation(i), 0.6);
end
axis tight; axis equal;

%Timeline plot of the state sequence probabilities
figure; hold on;
for i=1:hsmm.K
	patch([1, 1:N, N], [0, ht(i,:), 0], Morandi_carnation(i), ...
		'linewidth', 2, 'EdgeColor', max(Morandi_carnation(i)-0.2,0), 'facealpha', .6, 'edgealpha', .6);
end
set(gca,'xtick',(10:10:N),'fontsize',8); axis([1 N 0 1.1]);
xlabel('t','fontsize',16); 
ylabel('h_i','fontsize',16);
%}
