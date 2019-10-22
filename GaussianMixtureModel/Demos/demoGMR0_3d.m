%demoGMR0_3d 3D visualization of GMM with time-based GMR
%
% Haopeng Hu
% 2019.10.22
% All rights reserved

% load('Data\Letter3d.mat');

%% Data initialization
% We only need the positions

nSamples = 10;
dt = 1e-3;
nData = size(demos{1}.pos,2);   % Num. of data in each demo. Only position considered.
dData = size(demos{1}.pos,1);   % Dimension of the demo
Data = zeros(dData,nSamples*nData);
Demos = cell(1,nSamples); 
for i = 1:nSamples
    Data(:,(i-1)*nData+1:i*nData) = demos{i}.pos;   % Not used in Parameter estimation but in Figure
    Demos{i} = [(1:nData)'.*dt,(demos{i}.pos)'];    % Directly used in Parameter estimation, the 1st column is time
end

%% GMM initialization

gmm = GMMZero(5,dData+1,dt);

%% GMM param. estimation

gmm = gmm.initGMMKMeans(Demos);
gmm = gmm.learnGMM(Demos);

%% GMR

[expData, expSigma] = gmm.GMR((1:nData)'.*gmm.dt);

%% Figure

disp('Be patient ...');
figure; 
% %Plot GMM
subplot(1,2,1); hold on; box on; title('GMM');
gmm.plotGMM3SC([.8 0 0],.3);
for n=1:nSamples
	dTmp = [Data(1:3,(n-1)*nData+1:n*nData) fliplr(Data(1:3,(n-1)*nData+1:n*nData))];
	patch(dTmp(1,:),dTmp(2,:),dTmp(3,:), [.5,.5,.5],'facealpha',0,'linewidth',2,'edgecolor',[.5,.5,.5],'edgealpha',.5);
end
view(3); axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]); set(gca,'Ztick',[]);
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
grid on;
%Plot GMR
subplot(1,2,2); hold on; box on; title('GMR');
gmm.plotGMM3GMR(expData,expSigma,[0 .8 0],.2);
for n=1:nSamples
	dTmp = [Data(1:3,(n-1)*nData+1:n*nData) fliplr(Data(1:3,(n-1)*nData+1:n*nData))];
	patch(dTmp(1,:),dTmp(2,:),dTmp(3,:), [.5,.5,.5],'facealpha',0,'linewidth',2,'edgecolor',[.5,.5,.5],'edgealpha',.5);
end
plot3(expData(:,1),expData(:,2),expData(:,3),'-','linewidth',4,'color',[0 .4 0]);
view(3); axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]); set(gca,'Ztick',[]);
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
grid on;

%print('-dpng','graphs/demo_GMR_3Dviz01.png');
%pause;
%close all;
