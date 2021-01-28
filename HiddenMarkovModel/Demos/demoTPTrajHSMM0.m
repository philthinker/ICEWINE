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

hsmm = TPTrajHSMMZero(2,3,5,2);
hsmm = hsmm.initHMMKmeans(Demos);
hsmm = hsmm.initTransUniform();
hsmm = hsmm.learnHMM(Demos);

%% Demonstration retrival

N = 100;
[ht, seq] = hsmm.reconstructStSeq_StandardFW(N);
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

% for i = 1:4
%     Demos(i).A = zeros(6,6,2);
%     for j = 1:2
%         A = tmpDemos(i).A(2:3,2:3,j);
%         Demos(i).A(:,:,j) = blkdiag(A,A,A);
%     end
%     b = tmpDemos(i).b(2:3,:);
%     Demos(i).b = [b; zeros(4,2)];
%     Demos(i).data = tmpDemos(i).data(2:3,:);
%     Demos(i).data = [Demos(i).data; gradient(Demos(i).data(1:2,:))/0.01]; Demos(i).data(3:4,1) = [0 0]';
%     Demos(i).data = [Demos(i).data; gradient(Demos(i).data(3:4,:))/0.01]; Demos(i).data(5:6,1) = [0,0]';
%     N = size(Demos(i).data,2);
%     Demos(i).TPData = zeros(6,2,N);
%     for j = 1:N
%         for k = 1:2
%             A = Demos(i).A(:,:,k);
%             b = Demos(i).b(:,k);
%             Demos(i).TPData(:,k,j) = A\(Demos(i).data(:,j) - b);
%         end
%     end
% end

