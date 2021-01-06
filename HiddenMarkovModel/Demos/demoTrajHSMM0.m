%demoTrajHSMM0
%   Trajectory Hidden semi-Markov Model demo
%   Model learning and state sequence reconstruction.
%
%   Haopeng Hu
%   2019.11.27
%   All rights reserved
%
% Variable duration model implemented as a hidden semi-Markov model 
% (simplified version by encoding the state duration after EM).
% Trajectory synthesis with an HSMM with dynamic features (trajectory-HSMM).

%% Initialization

M = 6;
N = 100;
DP = 2; % dim. of position
DD = 3; % order of derivation
dt = 1;
hsmm = TrajHSMMZero(DP,DD,4);

%% Load handwriting data

load('./HiddenMarkovModel/Demos/Data/letterSPVA.mat');
for n = 1:M
    Data = Demos{n};
    Data = Data(1:DD*DP,:);
    Demos{n} = Data;
end

%% Learning

hsmm = hsmm.initHMMKbins(Demos);
hsmm = hsmm.initTransLeftRight(N);
hsmm = hsmm.leanHMM(Demos);

%% Reconstruction of states probability sequence

[ht, seq] = hsmm.reconstructStSeq_StandardFW(N);

%% Construction of the trajectory

[traj, trajSigma] = hsmm.constructTraj_lscov(seq,dt);


%% Plot 2D

figure;
hold on;
plotGMM2SC(hsmm.Mu([1,2],:), hsmm.Sigma([1,2],[1,2],:), [.5 .5 .5],.8);
plotGMM2SC(traj([1,2],:), trajSigma([1,2],[1,2],:), [1 .2 .2],.1);
for n=1:M
    Data = Demos{n};
    plot(Data(1,:), Data(2,:), '-','lineWidth',1,'color',[.2 .2 .2]);
end

plot(traj(1,:), traj(2,:), '-','lineWidth',3.5,'color',[.8 0 0]);
set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
xlabel(['$x_1$'],'interpreter','latex','fontsize',18);
ylabel(['$x_2$'],'interpreter','latex','fontsize',18);

