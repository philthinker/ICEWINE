%demoHMM0
%   Hidden semi-Markov Model demo
%
%   Haopeng Hu
%   2019.11.27
%   All rights reserved
%
% Variable duration model implemented as a hidden semi-Markov model 
% (simplified version by encoding the state duration after EM).

addpath('./CELLAR/pbdlib-matlab-master/demos/m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = [];
model.nbStates = 5; %Number of hidden states in the HSMM
nbData = 100; %Length of each trajectory
nbSamples = 10; %Number of demonstrations
minSigmaPd = 1E1; %Minimum variance of state duration (regularization term)

%%%%
M = 10;
N = 100;
hsmm = HSMMZero(2,5);

%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('./CELLAR/pbdlib-matlab-master/demos/data/2Dletters/V.mat');
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	s(n).nbData = size(s(n).Data,2);
	Data = [Data s(n).Data]; 
end


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_kmeans(Data, model);
model = init_GMM_kbins(Data, model, nbSamples);

%%%%
hsmm = hsmm.initHMMKbins(Demos);

% %Random initialization
% model.Trans = rand(model.nbStates,model.nbStates);
% model.Trans = model.Trans ./ repmat(sum(model.Trans,2),1,model.nbStates);
% model.StatesPriors = rand(model.nbStates,1);
% model.StatesPriors = model.StatesPriors/sum(model.StatesPriors);

%Left-right model initialization
model.Trans = zeros(model.nbStates);
for i=1:model.nbStates-1
	model.Trans(i,i) = 1-(model.nbStates/nbData);
	model.Trans(i,i+1) = model.nbStates/nbData;
end
model.Trans(model.nbStates,model.nbStates) = 1.0;
model.StatesPriors = zeros(model.nbStates,1);
model.StatesPriors(1) = 1;
model.Priors = ones(model.nbStates,1);

%%%%
hsmm = hsmm.initTransLeftRight(N);

[model, H] = EM_HMM(s, model);

%Removal of self-transition (for HSMM representation) and normalization
model.Trans = model.Trans - diag(diag(model.Trans)) + eye(model.nbStates)*realmin;
model.Trans = model.Trans ./ repmat(sum(model.Trans,2),1,model.nbStates);

%%%%
hsmm = hsmm.leanHMM(Demos);


%% Post-estimation of the state duration from data (for HSMM representation)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:model.nbStates
	st(i).d=[];
end
[~,hmax] = max(H);
currState = hmax(1);
cnt = 1;
for t=1:length(hmax)
	if (hmax(t)==currState)
		cnt = cnt+1;
	else
		st(currState).d = [st(currState).d cnt];
		cnt = 1;
		currState = hmax(t);
	end
end
st(currState).d = [st(currState).d cnt];

%Compute state duration as Gaussian distribution
for i=1:model.nbStates
	model.Mu_Pd(1,i) = mean(st(i).d);
	model.Sigma_Pd(1,1,i) = cov(st(i).d) + minSigmaPd;
end

% %dm=P(d) for each state can be computed with:
% rho = (nbData - sum(squeeze(model.Mu_Pd))) / sum(squeeze(model.Sigma_Pd));
% dm = squeeze(model.Mu_Pd) + (squeeze(model.Sigma_Pd)' * rho);

% model.Sigma_Pd(1,1,2:3) = 1E-1;

%% Reconstruction of states probability sequence
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbD = round(2 * nbData/model.nbStates); %Number of maximum duration step to consider in the HSMM (2 is a safety factor)

%Precomputation of duration probabilities 
for i=1:model.nbStates
	model.Pd(i,:) = gaussPDF([1:nbD], model.Mu_Pd(:,i), model.Sigma_Pd(:,:,i)); 
	%The rescaling formula below can be used to guarantee that the cumulated sum is one (to avoid the numerical issues)
	model.Pd(i,:) = model.Pd(i,:) / sum(model.Pd(i,:));
end


%Slow reconstruction of states sequence based on standard computation
%(in the iteration, a scaling factor c is used to avoid numerical underflow issues in HSMM, see Levinson'1986) 
h = zeros(model.nbStates,nbData);
c = zeros(nbData,1); %scaling factor to avoid numerical issues
c(1) = 1; %Initialization of scaling factor
for t=1:nbData
	for i=1:model.nbStates
		if t<=nbD
% 			oTmp = 1; %Observation probability for generative purpose
			oTmp = prod(c(1:t) .* gaussPDF(s(1).Data(:,1:t), model.Mu(:,i), model.Sigma(:,:,i))'); %Observation probability for standard HSMM
			h(i,t) = model.StatesPriors(i) * model.Pd(i,t) * oTmp;
		end
		for d=1:min(t-1,nbD)
% 			oTmp = 1; %Observation probability for generative purpose
			oTmp = prod(c(t-d+1:t) .* gaussPDF(s(1).Data(:,t-d+1:t), model.Mu(:,i), model.Sigma(:,:,i))'); %Observation probability for standard HSMM	
			h(i,t) = h(i,t) + h(:,t-d)' * model.Trans(:,i) * model.Pd(i,d) * oTmp;
		end
	end
	c(t+1) = 1/sum(h(:,t)+realmin); %Update of scaling factor
end
h = h ./ repmat(sum(h,1),model.nbStates,1);


% %Fast reconstruction of sequence for HSMM (version based on position and duration information)
% h = zeros(model.nbStates,nbData);
% [bmx, ALPHA, S, h(:,1)] = hsmm_fwd_init_hsum(s(1).Data(:,1), model);
% for t=2:nbData
% 	[bmx, ALPHA, S, h(:,t)] = hsmm_fwd_step_hsum(s(1).Data(:,t), model, bmx, ALPHA, S);
% end


% %Fast reconstruction of sequence for HSMM (version based on only duration information)
% h = zeros(model.nbStates,nbData);
% [ALPHA, S, h(:,1)] = hsmm_fwd_init_ts(model);
% for t=2:nbData
% 	[ALPHA, S, h(:,t)] = hsmm_fwd_step_ts(model, ALPHA, S);
% end
% h = h ./ repmat(sum(h,1),model.nbStates,1);


% %Manual reconstruction of sequence for HSMM based on stochastic sampling 
% nbSt=0; currTime=0; iList=[];
% h = zeros(model.nbStates,nbData);
% while currTime<nbData
% 	nbSt = nbSt+1;
% 	if nbSt==1
% 		[~,iList(1)] = max(model.StatesPriors.*rand(model.nbStates,1));
% 		h1 = ones(1,nbData);
% 	else
% 		h1 = [zeros(1,currTime), cumsum(model.Pd(iList(end-1),:)), ones(1,max(nbData-currTime-nbD,0))];
% 		currTime = currTime + round(model.Mu_Pd(1,iList(end-1)));
% 	end
% 	h2 = [ones(1,currTime), 1-cumsum(model.Pd(iList(end),:)), zeros(1,max(nbData-currTime-nbD,0))];
% 	h(iList(end),:) = h(iList(end),:) + min([h1(1:nbData); h2(1:nbData)]);
% 	[~,iList(end+1)] = max(model.Trans(iList(end),:).*rand(1,model.nbStates));
% end
% h = h ./ repmat(sum(h,1),model.nbStates,1);

%
%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,600],'color',[1 1 1]); 
clrmap = lines(model.nbStates);
%Spatial plot of the data
subplot(3,2,[1,3]); axis off; hold on; 
plot(Data(1,:), Data(2,:), '.', 'color', [.3 .3 .3]);
for i=1:model.nbStates
	plotGMM(model.Mu(:,i), model.Sigma(:,:,i), clrmap(i,:), .6);
end
axis tight; axis equal;
%HSMM transition and state duration plot
subplot(3,2,[2,4]); axis off; hold on; 
plotHSMM(model.Trans, model.StatesPriors, model.Pd);
axis([-1 1 -1 1]*1.9);
%Timeline plot of the state sequence probabilities
subplot(3,2,[5,6]); hold on;
for i=1:model.nbStates
	patch([1, 1:nbData, nbData], [0, h(i,:), 0], clrmap(i,:), ...
		'linewidth', 2, 'EdgeColor', max(clrmap(i,:)-0.2,0), 'facealpha', .6, 'edgealpha', .6);
end
set(gca,'xtick',[10:10:nbData],'fontsize',8); axis([1 nbData 0 1.1]);
xlabel('t','fontsize',16); 
ylabel('h_i','fontsize',16);
%}
