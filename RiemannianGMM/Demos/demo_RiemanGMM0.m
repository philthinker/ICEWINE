%demo_RiemanGMM0
%   Demo for usage of class @RiemannianGMMZero
%   
%   Haopeng Hu
%   2020.07.13
%   All rights reserved
%

%% Data init.

% % load('Data\pbdlib-UnitQuatS.mat')

M = 4;
N = 50;

%% Init. and learn GMM

% Init. GMM
gmm = RiemannianGMMZero(5,4);   % K, D. You must include query variable in D
% Init. data struct
Data = gmm.constructRiemannianData(xOut,uOut,xIn);
% Init. param. by K-Bins
gmm = gmm.initGMMKBins([Data.queryIn;Data.data],M,N);
% Learn param. by EM
[gmm,uTmp] = gmm.learnGMM(Data);

%% Retrieve demos (No generalization ability)

[Data.expDataM, Data.expData, Data.expSigma] = gmm.GMR(Data.queryIn);

%% Figure

clrmap = lines(gmm.nKernel);

%Timeline plot
figure;
for k=1:4
	subplot(2,2,k); hold on; 
	for n=1:M
		plot(x(1,(n-1)*N+1:n*N), x(1+k,(n-1)*N+1:n*N), '-','color',[.6 .6 .6]);
	end
	plot(x(1,1:N), Data.expDataM(k,1:N), '-','linewidth',2,'color',[.8 0 0]);
	xlabel('t'); ylabel(['q_' num2str(k)]);
end

%Tangent space plot
figure;
for i=1:gmm.nKernel
	subplot(ceil(gmm.nKernel/2),2,i); hold on; axis off; title(['k=' num2str(i) ', output space']);
	plot(0,0,'+','markersize',40,'linewidth',1,'color',[.7 .7 .7]);
 	plot(uTmp(2,:,i), uTmp(3,:,i), '.','markersize',4,'color',[0 0 0]);
	plotGMM2SC(gmm.Mu(2:3,i), gmm.Sigma(2:3,2:3,i)*3, clrmap(i,:), .3);
	axis equal;
end
