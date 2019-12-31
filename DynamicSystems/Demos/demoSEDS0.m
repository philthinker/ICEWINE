%demoSEDS0

% Haopeng Hu
% 2019.12.25 Merry Christmas!
% All rights reserved

%% Data init.

% % load('Data\LetterS.mat');
% 
M = 10;
N = 100;
dt = 0.01;

Data = zeros(4,M*N);
Data0 = zeros(2,M);
for i = 1:M
    tmpData = spline((1:size(demos{i}.pos,2)), demos{i}.pos, linspace(1,size(demos{i}.pos,2),N));
    tmpData = tmpData - repmat(tmpData(:,end),[1,N]);   % Centered at 0
    Data0(:,i) = tmpData(:,1);
    tmpVel = gradient(tmpData)/dt;  % Compute velocities
    Data(:,(i-1)*N+1:i*N) = [tmpData; tmpVel];
end
Data0 = mean(Data0,2);

%% Init. SEDS

seds = SEDSZero(4,6,dt);
seds = seds.preOptim(Data,M,N);
seds = seds.Refine(Data);

%% Reproduction

% Standard GMR
expDataGMR = seds.GMR(Data0,N);

% Stable SEDS dynamic programming
expDataDP = seds.DP(Data0,3*N);

%% Figure

figure;
for i = 1:M
    plot(Data(1,(i-1)*N+1:i*N), Data(2,(i-1)*N+1:i*N), '-','lineWidth',1,'color',[0.4 0.4 0.4]);
    hold on;
end
plotGMM2SC(seds.Mu(1:2,:),seds.Sigma(1:2,1:2,:),[0.6 0.6 0.6],0.8);
hl(1)=plot(expDataGMR(1,:), expDataGMR(2,:), '-','lineWidth',2,'color',[1 0 0]);
hl(2)=plot(expDataDP(1,:), expDataDP(2,:), '-','lineWidth',2,'color',[0 .8 0]);
legend(hl,'GMR reproduction','Stable SEDS reproduction');
axis equal;
