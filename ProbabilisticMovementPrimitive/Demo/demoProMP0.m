%demoProMP0 
%   A demo for the usage of ProMPZero
%
%   Haopeng Hu
%   2020.01.02
%   All rights reserved

%% Data init.

% load('Data\LetterS');
M = 9;
N = 200;
D = 2;
Demos = [];
Demos.data = demos{1}.pos; Demos = repmat(Demos,[1,M]);
for i = 1:M
    Demos(i).data = demos{i}.pos;
end
% Note that we DO NOT take the velocity into consideration in this demo.

%% Learn a ProMP

model = ProMPZero(D,20);
model = model.leanLRR(Demos);

%% Modulation

% goalData.t = 0.1;
% goalData.data = [0,0]';
% model = model.modulate(goalData);

%% Reproduction

[expData,expSigma] = model.reproduct(N);

t = linspace(0,1,N);
figure;
for i = 1:M
    plot(Demos(i).data(1,:),Demos(i).data(2,:),'Color',[0.5 0.5 0.5]);
    hold on;
end
plot(expData(1,:),expData(2,:),'Color',[0 0 1]);
plotGMM2SC(expData,expSigma,[235/255,104/255,119/255],0.2);
axis equal;
grid on;

figure;
model.plotBasis();

