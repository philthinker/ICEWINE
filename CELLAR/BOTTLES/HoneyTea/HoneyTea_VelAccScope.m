%HoneyTea_VelAccScope

%   Haopeng Hu
%   2020.01.08
%   All rights reserved

%% Test data

% ax = Data0108(:,9);
% vx = Data0108(:,5);
% px = Data0108(:,2);
% t = Data0108(:,1);

%% GPR

% % Data = [t';ax'];
% gp = GPZero(Data);
% gp = gp.setParam(1e0,1e-1,1e-8);
% gp = gp.preGPR();
% 
% axGPR = gp.GPR(linspace(Data(1,1),Data(1,end),500));
% axExp = axGPR.Data;

figure;
plot(Data(1,:),Data(2,:));
hold on;
plot(axExp(1,:),axExp(2,:));
