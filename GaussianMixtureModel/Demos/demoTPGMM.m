%demoTPGMM
%   A demo for TP-GMM
%
%   Haopeng Hu
%   2019.10.23
%   All rights reserved

%% Data Initialization
% Here what we need is the pose of each frame and corresponded data.
% For convenience, the original demo data is assumed to be recorded
% according to the 'world frame', i.e. A = I and b = 0.
% A demo struct contains:
%   A: D x D x F, rotation matrices
%   b: D x F, positions
%   data: N x D, demo data
%   TPData: D x F x N, demo data in each frame

% load('Data\Data01_2d2frame.mat');

%% TP-GMM learn

gmm = TPGMM(3,2,2); % 3 Gaussians, 2d data and 2 frames
gmm = gmm.initGMMKMeans(Demos);
gmm = gmm.learnGMM(Demos);

%% Figure

% %Reconstruct GMM for each demonstration
TPDemo = Demos;
for i=1:size(Demos,2)
    [TPDemo{i}.Mu,TPDemo{i}.Sigma] = gmm.productTPGMM(Demos{i});
    TPDemo{i}.data = TPDemo{i}.data';
    TPDemo{i}.nData = size(TPDemo{i}.data,2);
end

% Plot
figure();
xx = round(linspace(1,64,size(Demos,2)));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
limAxes = [-1.2 0.8 -1.1 0.9];
colPegs = [[.9,.5,.9];[.5,.9,.5]];

%DEMOS
subplot(1,gmm.nFrame+1,1); hold on; box on; title('Demonstrations');
for n=1:size(TPDemo,2)
	%Plot frames
	for m=1:gmm.nFrame
		plot([TPDemo{n}.b(1,m) TPDemo{n}.b(1,m)+TPDemo{n}.A(1,2,m)], [TPDemo{n}.b(2,m) TPDemo{n}.b(2,m)+TPDemo{n}.A(2,2,m)], '-','linewidth',6,'color',colPegs(m,:));
		plot(TPDemo{n}.b(1,m), TPDemo{n}.b(2,m),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
    plot(TPDemo{n}.data(1,1), TPDemo{n}.data(2,1),'.','markersize',15,'color',clrmap(n,:));
    plot(TPDemo{n}.data(1,:), TPDemo{n}.data(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
	%Plot Gaussians
	plotGMM2SC(TPDemo{n}.Mu, TPDemo{n}.Sigma, [.5 .5 .5],.8);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%FRAMES
demoData = gmm.tpDataRegulate(Demos);
for m=1:gmm.nFrame
	subplot(1,gmm.nFrame+1,1+m); hold on; grid on; box on; title(['Frame ' num2str(m)]);
	for n=1:size(TPDemo,2)
		plot(squeeze(demoData(1,m,(n-1)*TPDemo{1}.nData+1)), ...
			squeeze(demoData(2,m,(n-1)*TPDemo{1}.nData+1)), '.','markersize',15,'color',clrmap(n,:));
		plot(squeeze(demoData(1,m,(n-1)*TPDemo{1}.nData+1:n*TPDemo{1}.nData)), ...
			squeeze(demoData(2,m,(n-1)*TPDemo{1}.nData+1:n*TPDemo{1}.nData)), '-','linewidth',1.5,'color',clrmap(n,:));
	end
	plotGMM2SC(squeeze(gmm.Mus(:,m,:)), squeeze(gmm.Sigmas(:,:,m,:)), [.5 .5 .5],.8);
	axis square; set(gca,'xtick',[0],'ytick',[0]);
end
