%main_HSMM_0806XZ_TP0
%   HSMM LfD
%   HSMM (@LfDHSMMZero) with resampled & computed data.
%   Orientation does not enter into account.
%   - Adaptive initial state distribution.
%   - TrajHSMM formulation
%   - Accessible num. of data points in trajectories.
%   - Accessible goal distribution.
%
%   Haopeng Hu
%   2021.01.15
%   All rights reserved
%
%   Exp. with 'Data\pcb_lead0806XZV.mat'

%% Data resample
%
DD = 2;
DataXZAppResam = DataXZApp;
for i = 1:M
    tmpData = DataXZApp(i).p;
    [DataXZAppResam(i).p, tmpLogi] = THDResample(tmpData,0.000002);
    DataXZAppResam(i).p = slideAverageWindow(DataXZAppResam(i).p, 4);
    DataXZAppResam(i).SO2 = DataXZApp(i).SO2(:,:,tmpLogi);
    DataXZAppResam(i).xi = constructDynaData(DataXZAppResam(i).p,DD,dt);
end

% figure;
% hold on;
% for i = 1:M
%     tmpData = DataXZApp(i).p;
%     plot(tmpData(1,:), tmpData(2,:), 'Color', [0.8,0.1,0]);
%     hold on;
%     tmpData = DataXZAppResam(i).p;
%     plot(tmpData(1,:),tmpData(2,:),'Color',[0, 0.1, 0.8]);
% end
% axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');

% figure;
% for i = 1:M
%     tmpData = DataXZAppResam(i).xi(3:4,:);
%     tmpTime = (1:size(tmpData,2))*dt - dt;
%     subplot(2,1,1);
%     plot(tmpTime, tmpData(1,:), 'Color', Morandi_carnation(i));
%     hold on; grid on; ylabel('vx(m/s)');
%     subplot(2,1,2);
%     plot(tmpTime, tmpData(2,:), 'Color', Morandi_carnation(i));
%     hold on; grid on; xlabel('t(s)'), ylabel('vz(m/s)');
% end
%}

%% Learn a HSMM model with raw p data
%
DemosPApp = extractDataFieldAsCell(DataXZAppResam,'xi');

policyApp = LfDHSMMZero(2,DD,7,0.1);
policyApp = policyApp.initHMMKmeans(DemosPApp);

% figure;
% hold on;
% for i = 1:M
%     tmpData = DemosPApp{i};
%     plot(tmpData(1,:), tmpData(2,:),'Color',[0.5,0.5,0.5]);
% end
% plotGMM2SC(policyApp.Mu(1:2,:),policyApp.Sigma(1:2,1:2,:),[0, 0.5, 0.5], 0.6);
% axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');

policyApp = policyApp.initTransUniform();
policyApp = policyApp.leanHMM(DemosPApp);

figure;
hold on;
for i = 1:M
    tmpData = DemosPApp{i};
    plot(tmpData(1,:), tmpData(2,:),'Color',[0.5,0.5,0.5]);
end
for i=1:policyApp.K
	plotGMM2SC(policyApp.Mu(1:2,i), policyApp.Sigma(1:2,1:2,i), Morandi_carnation(i), 0.6);
end
axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');
%}

%% Demonstration retrieve
%
N = 100;

[traj, trajSigma, ht, seq] = policyApp.constructTraj_AdaptInit0([0.4,0.1]', N);
% [traj, trajSigma] = policyApp.constructTraj_lscov(seq,dt);

% Cartesian plot
figure;
hold on;
for i =1:1
    plot(DataXZAppResam(i).xi(1,:), DataXZAppResam(i).xi(2,:),'Color',[0.5,0.5,0.5]);
end
plot(traj(1,:), traj(2,:), 'Color', Morandi_carnation(2),'LineWidth',2.0);
grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');

% Timeline plot
figure;
ylabels = {'x(m)', 'z(m)'};
for i = 1:2
    t = (1:N)*dt - dt;
    subplot(2,1,i);
    plot(t, traj(i,:));
    grid on; xlabel('t(s)'); ylabel(ylabels{i});
end

%Timeline plot of the state sequence probabilities
figure; 
hold on;
for i=1:policyApp.K
	patch([1, 1:N, N], [0, ht(i,:), 0], Morandi_carnation(i), ...
		'linewidth', 2, 'EdgeColor', max(Morandi_carnation(i)-0.2,0), 'facealpha', .6, 'edgealpha', .6);
end
set(gca,'xtick',(10:10:N),'fontsize',8); axis([1 N 0 1.1]);
grid on;
xlabel('t'); 
ylabel('h');
%}

%% Initial position generalization
%

%}
