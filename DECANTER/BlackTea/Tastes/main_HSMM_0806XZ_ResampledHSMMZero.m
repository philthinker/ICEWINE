%main_HSMM_0806XZ_ResampledHSMMZero
%   HSMM LfD
%   Typical HSMM (@HSMMZero) with resampled data.
%   Orientation does not enter into account.
%
%   Haopeng Hu
%   2021.01.13
%   All rights reserved
%
%   Exp. with 'Data\pcb_lead0806XZ.mat'

%% Data resample
%
DataXZAppResam = DataXZApp;
for i = 1:M
    tmpData = DataXZApp(i).p;
    [DataXZAppResam(i).p, tmpLogi] = THDResample(tmpData,0.00005);
    DataXZAppResam(i).SO2 = DataXZApp(i).SO2(:,:,tmpLogi);
end

figure;
hold on;
for i = 1:M
    tmpData = DataXZApp(i).p;
    plot(tmpData(1,:), tmpData(2,:), 'Color', [0.8,0.1,0]);
    hold on;
    tmpData = DataXZAppResam(i).p;
    plot(tmpData(1,:),tmpData(2,:),'Color',[0, 0.1, 0.8]);
end
axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');
%}

%% Learn a HSMM model with raw p data
%
DemosPApp = extractDataFieldAsCell(DataXZAppResam,'p');

policyApp = HSMMZero(2,6);
policyApp.dt = 0.01;
policyApp = policyApp.initHMMKmeans(DemosPApp);

figure;
hold on;
for i = 1:M
    tmpData = DemosPApp{i};
    plot(tmpData(1,:), tmpData(2,:),'Color',[0.5,0.5,0.5]);
end
plotGMM2SC(policyApp.Mu,policyApp.Sigma,[0, 0.5, 0.5], 0.6);
axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');

policyApp = policyApp.initTransUniform();
policyApp = policyApp.leanHMM(DemosPApp);

figure;
hold on;
for i = 1:M
    tmpData = DemosPApp{i};
    plot(tmpData(1,:), tmpData(2,:),'Color',[0.5,0.5,0.5]);
end
plotGMM2SC(policyApp.Mu,policyApp.Sigma,[0, 0, 0.5], 0.6);
axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');
%}

%% Reconstruct the forward sequence
%
N = 200;
[ht, seq] = policyApp.reconstructStSeq_StandardFW(N);

figure;
hold on; 
for i = 1:M
    tmpData = DemosPApp{i};
    plot(tmpData(1,:), tmpData(2,:),'Color',[0.3,0.3,0.3]);
end
for i=1:policyApp.K
	plotGMM2SC(policyApp.Mu(:,i), policyApp.Sigma(:,:,i), Morandi_carnation(i), 0.6);
end
axis equal; grid on; xlabel('x(m)'); ylabel('z(m)');

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