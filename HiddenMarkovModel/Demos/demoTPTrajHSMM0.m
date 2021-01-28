%demoTPTrajHSMM0
%   Demo for using @TPTrajHSMMZero
%   Task-parameterzied trajectory hidden semi-Markov model
%
%   Haopeng Hu
%   2021.01.27
%   All rights reserved
%
%   Data: 'Data\Data01_2d2frame.mat'

%% Load data

% load('.\HiddenMarkovModel\Demos\Data\Data01_2d2frame.mat');

%% Initialization

hsmm = TPTrajHSMMZero(2,3,2,2);
hsmm = hsmm.initHMMKmeans(Demos);
hsmm = hsmm.initHMMKmeansPosition(Demos);
hsmm = hsmm.initTransUniform();
hsmm = hsmm.learnHMM(Demos);

%% Demonstration retrival

N = 500; M = length(Demos);
[~, seq] = hsmm.reconstructStSeq_StandardFW(N);
traj = cell(1,M);
trajSigma = cell(1,M);
frames = [];
frames.A = [];
frames.b = [];
frames = repmat(frames,[1,hsmm.F]);
for i = 1:M
    for f = 1:hsmm.F
        frames(f).A = Demos(i).A(:,:,f);
        frames(f).b = Demos(i).b(:,f);
    end
    [traj{i}, trajSigma{i}] = hsmm.constructTraj_lscov(seq,frames);
end

%% Plot 2D

for f = 1:hsmm.F
    figure;
    hold on;
    for n=1:M
        tmpData = Demos(n).TPData(1:hsmm.DP, f, :);
        tmpData = permute(tmpData,[1,3,2]);
        plot(tmpData(1,:), tmpData(2,:), '-','lineWidth',1,'color',[.2 .2 .2]);
    end
    tmpMu = hsmm.Mus(1:hsmm.DP,f,:);
    tmpMu = permute(tmpMu, [1,3,2]);
    tmpSigma = hsmm.Sigmas(1:hsmm.DP, 1:hsmm.DP, f, :);
    tmpSigma = permute(tmpSigma,[1,2,4,3]);
    plotGMM2SC(tmpMu, tmpSigma, [1 .2 .2],.1);
    grid on;
    set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
    xlabel(['$x_1$'],'interpreter','latex','fontsize',18);
    ylabel(['$x_2$'],'interpreter','latex','fontsize',18);
end

figure;
hold on;
for n=1:M
    tmpData = Demos(n).data(1:hsmm.DP,:);
    plot(tmpData(1,:), tmpData(2,:), '-','lineWidth',1,'color',[.2 .2 .2]);
    tmpTraj = traj{n};
    plot(tmpTraj(1,:), tmpTraj(2,:), '-','lineWidth',3.5,'color',Morandi_carnation(n));
end
set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
xlabel(['$x_1$'],'interpreter','latex','fontsize',18);
ylabel(['$x_2$'],'interpreter','latex','fontsize',18);

