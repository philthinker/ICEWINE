%noodles1030

% MTD = MouseCaseAssembly('noodles1030');

% Data: Data1030_lead
% frankaData = MTD.frankaDataFormulate(frankaData);

%% Show raw data
%{
figure;
for i = 1:M
    tmpData = frankaData(i).p;
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color', Morandi_popsicle(i));
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

figure;
for i = 1:M
    tmpq = frankaData(i).q;
    tmpt = frankaData(i).time;
    for j = 1:4
        subplot(4,1,j);
        plot(tmpt,tmpq(j,:),'Color',Morandi_popsicle(i));
        hold on; grid on;
    end
end
%}

%% GMM based position policy for Panda robot

% Demos init.
%{
Demos_pPanda = cell(1,M);
for i = 1:M
    Demos_pPanda{i} = [frankaData(i).time; frankaData(i).p];
end
%}
% Policy learning
%{
ppolicy = GMMOne(6,4);
ppolicy = ppolicy.initGMMTimeBased(ppolicy.dataRegulate(Demos_pPanda));
ppolicy = ppolicy.learnGMM(ppolicy.dataRegulate(Demos_pPanda));
%}
% GMR
%{
query = linspace(0,1,1000);
[exp_pPanda, Sigma_pPanda] = ppolicy.GMR(query);
%}
% Show
%{
figure;
for i = 1:M
    tmpData = frankaData(i).p;
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', Morandi_popsicle(1));
    hold on;
end
plot3(exp_pPanda(1,:), exp_pPanda(2,:), exp_pPanda(3,:),'Color', Morandi_popsicle(2),'LineWidth',2.0);
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
MTD.pPolicy = ppolicy;
%}

%% QGMM based orientation policy for Panda robot

% Policy initialization
% qpolicy = QGMMZero(6,5,[1 0 0 0]');
% Demos init.
%{
Demos_qPanda = cell(1,M);
for i = 1:M
    Demos_qPanda{i} = [frankaData(i).time;qpolicy.logmap(frankaData(i).q)];
end
%}
% Policy learning
%{
qpolicy = qpolicy.initGMMTimeBased(qpolicy.dataRegulate(Demos_qPanda));
qpolicy = qpolicy.learnGMM(qpolicy.dataRegulate(Demos_qPanda));
%}
% GMR
%{
[exp_etaPanda, Sigma_etaPanda] = qpolicy.GMR(query);
exp_qPanda = qpolicy.expmap(exp_etaPanda);
%}
% Show
%{
ylabels = {'w','x','y','z'};
figure;
for j = 1:4
    subplot(4,1,j);
    for i = 1:M
        tmpData = frankaData(i).q;
        t = frankaData(i).time;
        plot(t, tmpData(j,:),'Color',Morandi_popsicle(1));
        hold on;
    end
    plot(query, exp_qPanda(j,:),'Color',Morandi_popsicle(2),'LineWidth',2.0);
    ylabel(ylabels{j}); grid on;
end

MTD.qPolicy = qpolicy;
%}

%% DOF assignment

% Data init. 
%{
exeData = [];
exeData.panda_p = exp_pPanda;
exeData.panda_pSigma = Sigma_pPanda;
exeData.panda_eta = exp_etaPanda;
exeData.panda_etaSigma = Sigma_etaPanda;
exeData.panda_q = exp_qPanda;
exeData.query = query;
%}
% Goals
%{
exeData.panda_pGoal = goalSE3(1:3,4);
exeData.panda_RGoal = goalSE3(1:3,1:3);
exeData.panda_qGoal = tform2quat(goalSE3)';
%}
% Data regulate
%{
exeData.panda_pReal = [exeData.panda_RGoal * exeData.panda_p + exeData.panda_pGoal, exeData.panda_pGoal];
exeData.panda_qReal = [ quatProduct(...
    repmat(exeData.panda_qGoal,...
    [1,size(exeData.query,2)]),...
    exeData.panda_q), exeData.panda_qGoal];
%}
% Show
%{
figure;
for i = 1:M
    tmpData = frankaData(i).OTEE(:,13:15);
    plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3), 'Color', Morandi_popsicle(1));
    hold on;
end
tmpData = exeData.panda_pReal;
plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color',Morandi_popsicle(2),'LineWidth',2.0);
grid on; axis equal;
view(3);

figure;
ylabels = {'w','x','y','z'};
for i = 1:M
    tmpData = tform2quat(frankaData(i).rawSE3);
    t = frankaData(i).time;
    for j = 1:4
        subplot(4,1,j);
        plot(t, tmpData(:,j),'Color',Morandi_popsicle(1));
        hold on; grid on; ylabel(ylabels{j});
    end
end
tmpData = exeData.panda_qReal;
t = [exeData.query,1];
for j = 1:4
    subplot(4,1,j);
    plot(t, tmpData(j,:), 'Color', Morandi_popsicle(2), 'LineWidth', 2.0);
end
%}
