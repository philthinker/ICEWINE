%main_HSMM_0806XZ_LQT0
%   HSMM LfD
%   HSMM (@LfDHSMMOne) with resampled & computed data.
%   Orientation does not enter into account.
%   Compare with TP-GMR
%   Compare with TP-TrajHSMM
%   Good Luck!
%
%   Haopeng Hu
%   2021.01.25
%   All rights reserved
%
%   Exp. with 'Data\pcb_lead0806XZV.mat'

%% Data resample
%{
DP = 2;
DD = 3;
dt = 0.01;
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
%{
DemosPApp = extractDataFieldAsCell(DataXZAppResam,'p');

policyApp = LfDHSMMOne(7,DP,DD);
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

%% Demonstration retrieval
%{
N = 120;

policyApp.r = 1e-2;

tmpID = 9;
traj = cell(1,M);
h = cell(1,M);
seq = cell(1,M);
for i = 1:M
    tmpP0 = DataXZAppResam(i).p(1:DP,1);
    tmpSID = policyApp.stateDetermine(tmpP0);
    tmpPolicy = policyApp.resetStatePrior(tmpSID);
    [h{i}, seq{i}] = tmpPolicy.reconstructStSeq_StandardFW(N);
    traj{i} = tmpPolicy.LQTIterative(tmpP0,seq{i});
end

% Cartesian plot
figure;
hold on;
for i =1:M
    plot(DataXZAppResam(i).p(1,:), DataXZAppResam(i).p(2,:),'Color',[0.5,0.5,0.5]);
    tmpTraj = traj{i};
plot(tmpTraj(1,:), tmpTraj(2,:), 'Color', Morandi_carnation(i),'LineWidth',2.0);
end
grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');

% Timeline plot
%{
figure;
ylabels = {'x(m)', 'z(m)'};
tmpTraj = traj{tmpID};
for i = 1:2
    t = (1:N)*dt - dt;
    subplot(2,1,i);
    plot(t, tmpTraj(i,:));
    grid on; xlabel('t(s)'); ylabel(ylabels{i});
end

%Timeline plot of the state sequence probabilities
figure; 
hold on;
ht = h{tmpID};
for i=1:policyApp.K
	patch([1, 1:N, N], [0, ht(i,:), 0], Morandi_carnation(i), ...
		'linewidth', 2, 'EdgeColor', max(Morandi_carnation(i)-0.2,0), 'facealpha', .6, 'edgealpha', .6);
end
set(gca,'xtick',(10:10:N),'fontsize',8); axis([1 N 0 1.1]);
grid on;
xlabel('t'); 
ylabel('h');
%}
%}

%% Initial position generalization
%{
tmpID = 9;

MG = length(DataXZAppGen) - M;
trajGen = cell(1,MG);
hGen = cell(1,MG);
seqGen = cell(1,MG);
uGen = cell(1,MG);
for i = 1:MG
    tmpP0 = DataXZAppGen(i+M).p0(1:DP,1);
    tmpSID = policyApp.stateDetermine(tmpP0);
    tmpPolicy = policyApp.resetStatePrior(tmpSID);
    [hGen{i}, seqGen{i}] = tmpPolicy.reconstructStSeq_StandardFW(N);
    [trajGen{i},uGen{i}] = tmpPolicy.LQTIterative(tmpP0,seqGen{i});
end

% Cartesian plot
figure;
hold on;
for i =1:M
    plot(DataXZAppResam(i).p(1,:), DataXZAppResam(i).p(2,:),'Color',[0.5,0.5,0.5]);
end
for i = 10:MG
    tmpTraj = trajGen{i};
    plot(tmpTraj(1,:), tmpTraj(2,:), 'Color', Morandi_carnation(i),'LineWidth',2.0);
end
grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');

% Timeline plot
%{
figure;
ylabels = {'ax(m/s2)', 'az(m/s2)'};
tmpTraj = uGen{tmpID};
for i = 1:2
    t = (1:N)*dt - dt;
    subplot(2,1,i);
    plot(t, tmpTraj(i,:));
    grid on; xlabel('t(s)'); ylabel(ylabels{i});
end

%Timeline plot of the state sequence probabilities
figure; 
hold on;
ht = hGen{tmpID};
for i=1:policyApp.K
	patch([1, 1:N, N], [0, ht(i,:), 0], Morandi_carnation(i), ...
		'linewidth', 2, 'EdgeColor', max(Morandi_carnation(i)-0.2,0), 'facealpha', .6, 'edgealpha', .6);
end
set(gca,'xtick',(10:10:N),'fontsize',8); axis([1 N 0 1.1]);
grid on;
xlabel('t'); 
ylabel('h');
%}
%}

%% TP-GMR formulation with DTW
%{
F = 2;
policyAppGMR = FWTPGMMZero(7,3,F);

% DTW
tmpDemosPAppDTW = DemosPApp;
for i = 1:M
    tmpData = DemosPApp{i};
    tmpN = size(tmpData,2);
    tmpDemosPAppDTW{i} = [ (0:tmpN-1)*dt; tmpData]';
end
tmpDemosPAppDTW = iceDTW(tmpDemosPAppDTW,10);
% Construct TP Demos
for i = 1:M
    tmpData = tmpDemosPAppDTW{i}';
    tmpA = repmat(eye(DP+1),[1,1,F]);
    tmpA(2:end,2:end,:) = DataXZAppResam(i).SO2(:,:,[1,end]);
    tmpb = [zeros(1,2); DataXZAppResam(i).p(:,[1,end])];    % Never forget the time variable
    DemosPAppDTW(i) = policyAppGMR.TPDemoConstruct(tmpData,tmpA,tmpb);
end

% Learn TP-GMM
policyAppGMR = policyAppGMR.initGMMTimeBased(DemosPAppDTW);
policyAppGMR = policyAppGMR.learnGMM(DemosPAppDTW);

% Demonstration retrieval

trajGMR = policyAppGMR.TPGMRDataConstruct();
trajGMR = repmat(trajGMR,[1,M]);
for i = 1:M
    tmpQuery = DemosPAppDTW(i).data(1,:);
    tmpA = DemosPAppDTW(i).A;
    tmpb = DemosPAppDTW(i).b;
    trajGMR(i) = policyAppGMR.TPGMRDataConstruct(tmpQuery, tmpA, tmpb);
    % TP-GMR
    [trajGMR(i).data, trajGMR(i).Sigma] = policyAppGMR.GMR(trajGMR(i).query, trajGMR(i).frames);
end

% Cartesian plot
figure;
hold on;
for i =1:M
    plot(DemosPAppDTW(i).data(2,:), DemosPAppDTW(i).data(3,:),'Color',[0.5,0.5,0.5]);
    tmpTraj = trajGMR(i).data;
    plot(tmpTraj(1,:), tmpTraj(2,:), 'Color', Morandi_carnation(i),'LineWidth',2.0);
end
grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');

% Initial position generalization

MG = length(DataXZAppGen);

trajGMR_Gen = policyAppGMR.TPGMRDataConstruct();
trajGMR_Gen = repmat(trajGMR_Gen, [1,MG]);
tmpQuery = DemosPAppDTW(i).data(1,:);
for i = 1:MG
    tmpA = repmat(eye(DP+1), [1,1,F]);
    tmpb = zeros(DP+1,F);
    tmpA(2:end,2:end,1) = DataXZAppGen(i).SO20;
    tmpA(:, :, 2) = DemosPAppDTW(1).A(:,:,2);
    tmpb(2:end,1) = DataXZAppGen(i).p0;
    tmpb(:,2) = DemosPAppDTW(1).b(:,2);
    trajGMR_Gen(i) = policyAppGMR.TPGMRDataConstruct(tmpQuery, tmpA, tmpb);
    % TP-GMR
    [trajGMR_Gen(i).data, trajGMR_Gen(i).Sigma] = policyAppGMR.GMR(trajGMR_Gen(i).query, trajGMR_Gen(i).frames);
end

% Cartesian plot
figure;
hold on;
for i =1:M
    plot(DemosPAppDTW(i).data(2,:), DemosPAppDTW(i).data(3,:),'Color',[0.5,0.5,0.5]);
end
for i = 19:MG
    tmpTraj = trajGMR_Gen(i).data;
    plot(tmpTraj(1,:), tmpTraj(2,:), 'Color', Morandi_carnation(i),'LineWidth',2.0);
end
grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');
%}

%% TP-trajHSMM formulation (Fail)
%{
F = 2;
policyAppTPTrajHSMM = TPTrajHSMMZero(2,3,7,2);
policyAppTPTrajHSMM.dt = dt;

% Construct dynamic TP data
DemosPAppDyna = repmat(policyAppTPTrajHSMM.TPDemoConstruct_Dynamic(), [1,M]);
for i = 1:M
    DemosPAppDyna(i) = policyAppTPTrajHSMM.TPDemoConstruct_Dynamic(...
                                        DemosPApp{i},...
                                        DemosPAppDTW(i).A(2:end,2:end,:),...
                                        DemosPAppDTW(i).b(2:end,:));
end

% Learn TPTrajHSMM
policyAppTPTrajHSMM = policyAppTPTrajHSMM.initHMMKmeans(DemosPAppDyna);
policyAppTPTrajHSMM = policyAppTPTrajHSMM.initTransUniform();
policyAppTPTrajHSMM = policyAppTPTrajHSMM.learnHMM(DemosPAppDyna);

[~,seq] = policyAppTPTrajHSMM.reconstructStSeq_StandardFW(1000);

% Demonstration retrieval

trajTPTraj =[];
trajTPTraj.data = [];
trajTPTraj.Sigma = [];
tmpFrame = [];
tmpFrame.A = [];
tmpFrame.b = [];
trajTPTraj.frames = repmat(tmpFrame,[1,2]);
trajTPTraj = repmat(trajTPTraj,[1,M]);
for i = 1:M
    for j = 1:policyAppTPTrajHSMM.F
        trajTPTraj(i).frames(j).A = DemosPAppDyna(i).A(:,:,j);
        trajTPTraj(i).frames(j).b = DemosPAppDyna(i).b(:,j);
    end
    [trajTPTraj(i).data, trajTPTraj(i).Sigma] = policyAppTPTrajHSMM.constructTraj_lscov(seq, trajTPTraj(i).frames);
end

% Cartesian plot
figure;
hold on;
for i =1:M
    plot(DemosPAppDyna(i).data(1,:), DemosPAppDyna(i).data(2,:),'Color',[0.5,0.5,0.5]);
    tmpTraj = trajTPTraj(i).data;
    plot(tmpTraj(1,:), tmpTraj(2,:), 'Color', Morandi_carnation(i),'LineWidth',2.0);
end
grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');

% Initial position generalization

% MG = length(DataXZAppGen);

% % Cartesian plot
% figure;
% hold on;
% for i =1:M
%     plot(DemosPAppDyna(i).data(1,:), DemosPAppDyna(i).data(2,:),'Color',[0.5,0.5,0.5]);
% end
% for i = 19:MG
%     tmpTraj = trajTraj_Gen(i).data;
%     plot(tmpTraj(1,:), tmpTraj(2,:), 'Color', Morandi_carnation(i),'LineWidth',2.0);
% end
% grid on; axis equal; xlabel('x(m)'); ylabel('z(m)');
%}

%% TP-LQT (iteratively)

policyAppTPLQT = TPTrajHSMMZero(2,1,7,2);


