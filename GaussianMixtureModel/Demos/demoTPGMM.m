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
% gmm = gmm.learnGMM(Demos);

%% Figure

% %Reconstruct GMM for each demonstration
% for n=1:nbSamples
% 	[s(n).Mu, s(n).Sigma] = productTPGMM0(model, s(n).p);
% end

% %Products of linearly transformed Gaussians
% for n=1:nbSamples
% 	for i=1:model.nbStates
% 		SigmaTmp = zeros(model.nbVar);
% 		MuTmp = zeros(model.nbVar,1);
% 		for m=1:model.nbFrames
% 			MuP = s(n).p(m).A * model.Mu(:,m,i) + s(n).p(m).b;
% 			SigmaP = s(n).p(m).A * model.Sigma(:,:,m,i) * s(n).p(m).A';
% 			SigmaTmp = SigmaTmp + inv(SigmaP);
% 			MuTmp = MuTmp + SigmaP\MuP;
% 		end
% 		s(n).Sigma(:,:,i) = inv(SigmaTmp);
% 		s(n).Mu(:,i) = s(n).Sigma(:,:,i) * MuTmp;
% 	end
% end

% % Plot
% figure('position',[20,50,1300,500]);
% xx = round(linspace(1,64,nbSamples));
% clrmap = colormap('jet');
% clrmap = min(clrmap(xx,:),.95);
% limAxes = [-1.2 0.8 -1.1 0.9];
% colPegs = [[.9,.5,.9];[.5,.9,.5]];
% 
% %DEMOS
% subplot(1,model.nbFrames+1,1); hold on; box on; title('Demonstrations');
% for n=1:nbSamples
% 	%Plot frames
% 	for m=1:model.nbFrames
% 		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
% 		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
% 	end
% 	%Plot trajectories
% 	plot(s(n).Data(2,1), s(n).Data(3,1),'.','markersize',15,'color',clrmap(n,:));
% 	plot(s(n).Data(2,:), s(n).Data(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
% 	%Plot Gaussians
% 	plotGMM(s(n).Mu, s(n).Sigma, [.5 .5 .5],.8);
% end
% axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);
% 
% %FRAMES
% for m=1:model.nbFrames
% 	subplot(1,model.nbFrames+1,1+m); hold on; grid on; box on; title(['Frame ' num2str(m)]);
% 	for n=1:nbSamples
% 		plot(squeeze(Data(1,m,(n-1)*s(1).nbData+1)), ...
% 			squeeze(Data(2,m,(n-1)*s(1).nbData+1)), '.','markersize',15,'color',clrmap(n,:));
% 		plot(squeeze(Data(1,m,(n-1)*s(1).nbData+1:n*s(1).nbData)), ...
% 			squeeze(Data(2,m,(n-1)*s(1).nbData+1:n*s(1).nbData)), '-','linewidth',1.5,'color',clrmap(n,:));
% 	end
% 	plotGMM(squeeze(model.Mu(:,m,:)), squeeze(model.Sigma(:,:,m,:)), [.5 .5 .5],.8);
% 	axis square; set(gca,'xtick',[0],'ytick',[0]);
% end
