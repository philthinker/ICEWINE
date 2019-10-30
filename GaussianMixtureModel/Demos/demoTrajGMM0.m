%demoTrajGMM0
%   Trajectory synthesis with a GMM with dynamic features (trajectory GMM).
%
%   Haopeng Hu
%   2019.10.24
%   All rights reserved

%% Data and GMM initialization

% load('Data\LetterS.mat');
nSample = 4;    % Num. of demos needed
nData = 100;    % Num. of data in one trajectory
gmm = TrajGMM(5,2,3,0.001,nSample,nData);   % nKernel, nVarPos, nDeriv, dt, nSample, nData
% % Resampling
Demos = cell(1,nSample);
for n=1:nSample
	Demos{n} = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nData)); %Resampling
end
[Data1,x1,zeta1] = gmm.dynamicDataGeneration(Demos);

%% Learn GMM param.

% gmm = gmm.initGMMKMeans(Data1);
gmm = gmm.initGMMTimeBased_TmpTime(Data1,nData);
[gmm,GAMMA] = gmm.learnGMM(Data1);

%% Reproduction by trajectory-GMM

GAMMAReprod = GAMMA(:,1:nData); % We query the 1st demo
query = gmm.deriveQuery(GAMMAReprod);
[expData,expSigma] = gmm.reproductTrajGMM(query);

%% Figure

% Plot timeline
figure;
for m=1:gmm.nVarPos
	limAxes = [1, nData, min(Data(m,:))-1E0, max(Data(m,:))+1E0];
	subplot(gmm.nVarPos,1,m); hold on;
	for n=1:1 %nbSamples
		msh=[]; x0=[];
		for t=1:nData-1
			if size(msh,2)==0
				msh(:,1) = [t; gmm.Mu(m,query(t))];
			end
			if t==nData-1 || query(t+1)~= query(t)
				msh(:,2) = [t+1; gmm.Mu(m,query(t))];
				sTmp = gmm.Sigma(m,m,query(t))^.5;
				msh2 = [msh(:,1)+[0;sTmp], msh(:,2)+[0;sTmp], msh(:,2)-[0;sTmp], msh(:,1)-[0;sTmp], msh(:,1)+[0;sTmp]];
				patch(msh2(1,:), msh2(2,:), [.85 .85 .85],'edgecolor',[.7 .7 .7]);
				plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',[.7 .7 .7]);
				plot([msh(1,1) msh(1,1)], limAxes(3:4), ':','linewidth',1,'color',[.7 .7 .7]);
				x0 = [x0 msh];
				msh=[];
			end
		end
		msh = [1:nData, nData:-1:1; expData(m,:)-squeeze(r(n).expSigma(m,m,:).^.5)'*1, fliplr(expData(m,:)+squeeze(r(n).expSigma(m,m,:).^.5)'*1)];
		patch(msh(1,:), msh(2,:), [1 .4 .4],'edgecolor',[1 .2 .2],'edgealpha',.8,'facealpha',.5);
	end
	for n=1:nSample
		plot(1:nData, Data(m,(n-1)*nData+1:n*nData), '-','lineWidth',1,'color',[.2 .2 .2]);
	end
	for n=1:1
		plot(1:nData, expData(m,:), '-','lineWidth',2.5,'color',[.8 0 0]);
	end
	
	set(gca,'xtick',[],'ytick',[]);
	xlabel('$t$','interpreter','latex','fontsize',18);
	ylabel(['$x_' num2str(m) '$'],'interpreter','latex','fontsize',18);
	axis(limAxes);
end

% Plot 2D
if gmm.nVarPos>1
	figure; hold on;
	for n=1:1 %nbSamples
		plotGMM2SC(expData([1,2],:), expSigma([1,2],[1,2],:), [1 .2 .2],.2);
	end
	plotGMM2SC(gmm.Mu([1,2],:), gmm.Sigma([1,2],[1,2],:), [.5 .5 .5],.8);
	for n=1:nSample
		plot(Data(1,(n-1)*nData+1:n*nData), Data(2,(n-1)*nData+1:n*nData), '-','lineWidth',1,'color',[.2 .2 .2]); %-0.2+0.8*(n-1)/(nbSamples-1)
	end
	for n=1:1
		plot(expData(1,:), expData(2,:), '-','lineWidth',2.5,'color',[.8 0 0]);
	end
	set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
	xlabel(('$x_1$'),'interpreter','latex','fontsize',18);
	ylabel(('$x_2$'),'interpreter','latex','fontsize',18);
end