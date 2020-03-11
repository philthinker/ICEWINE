%demoTPTrajGMM1 A demo for class @TrajGMMOne with multiple frames
%
% Haopeng Hu
% 2020.03.10
% All rights reserved
%

%% Load TP data

% For TP-Traj-GMM:
% load('Data\Data04_2d2frame.mat');
%   TPDemo struct:
%   |   data: DPos x N, demo data
%   |   A: D x D x F, orientation matrices
%   |   b: D x F, position vectors
%   |   TPData: D x F x N, demo data in each frame

%% Init. TP-Traj-GMM

tpModel = TrajGMMOne(3,2,3,2);
tpModel = tpModel.initGMMKMeans2EM_Pos(Demos);
%model = init_tensorGMM_timeBased(Data, model); %Initialization
%model = init_tensorGMM_kmeans(Data, model); %Initialization

%%%% ???? %%%%
model = EM_tensorGMM(Data, model);

%% Reproduction for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions...');
for n=1:nbSamples
	%Products of linearly transformed Gaussians
	for i=1:model.nbStates
		SigmaTmp = zeros(model.nbVar);
		MuTmp = zeros(model.nbVar,1);
		for m=1:model.nbFrames
			MuP = s(n).p(m).A * model.Mu(:,m,i) + s(n).p(m).b;
			SigmaP = s(n).p(m).A * model.Sigma(:,:,m,i) * s(n).p(m).A';
			SigmaTmp = SigmaTmp + inv(SigmaP);
			MuTmp = MuTmp + SigmaP\MuP;
		end
		r(n).Sigma(:,:,i) = inv(SigmaTmp);
		r(n).Mu(:,i) = r(n).Sigma(:,:,i) * MuTmp;
	end
	%Create single Gaussian N(MuQ,SigmaQ) based on state sequence q, see Eq. (27)
	[~,r(n).q] = max(model.Pix(:,(n-1)*nbData+1:n*nbData),[],1); %works also for nbStates=1
	r(n).MuQ = reshape(r(n).Mu(:,r(n).q), model.nbVarPos*model.nbDeriv*nbData, 1);
	r(n).SigmaQ = zeros(model.nbVarPos*model.nbDeriv*nbData);
	for t=1:nbData
		id1 = (t-1)*model.nbVarPos*model.nbDeriv+1:t*model.nbVarPos*model.nbDeriv;
		r(n).SigmaQ(id1,id1) = r(n).Sigma(:,:,r(n).q(t));
	end
	%Retrieval of data with trajectory GMM, see Eq. (30)
	PHIinvSigmaQ = PHI1'/r(n).SigmaQ;
	Rq = PHIinvSigmaQ * PHI1;
	rq = PHIinvSigmaQ * r(n).MuQ;
	r(n).Data = reshape(Rq\rq, model.nbVarPos, nbData); %Reshape data for plotting
end

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
xx = round(linspace(1,64,nbSamples));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
limAxes = [-1.2 0.8 -1.1 0.9];
colPegs = [[.9,.5,.9];[.5,.9,.5]];

%DEMOS
subplot(1,3,1); hold on; box on; title('Demonstrations');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data0(1,1), s(n).Data0(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(s(n).Data0(1,:), s(n).Data0(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%REPROS
subplot(1,3,2); hold on; box on; title('Reproductions');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbSamples
	%Plot trajectories
	plot(r(n).Data(1,1), r(n).Data(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(r(n).Data(1,:), r(n).Data(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
for n=1:nbSamples
	%Plot Gaussians
	plotGMM(r(n).Mu(1:2,:,1), r(n).Sigma(1:2,1:2,:,1), [.5 .5 .5], .4);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);
