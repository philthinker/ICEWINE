%typical_tpgmm_test

% Load Data\Data1017.mat

%% Show X Y Z via plot
%{
figure;
labels = {'x(m)','y(m)','z(m)'};
for i = 1:N
    tmpT = (1:size(Data(i).OTEE_app,1))*dt - dt;
    for j = 1:3
        subplot(3,1,j);
        plot(tmpT,Data(i).OTEE_app(:,12+j));
        hold on;
        ylabel(labels{j});
        grid on;   
    end
end
%}
%{
figure;
labels = {'x(m)','y(m)','z(m)'};
for i = 1:N
    tmpT = (1:size(Data(i).OTEE_ass,1))*dt - dt;
    for j = 1:3
        subplot(3,1,j);
        plot(tmpT,Data(i).OTEE_ass(:,12+j));
        hold on;
        ylabel(labels{j});
        grid on;   
    end
end
%}

%% Down Sampling
%{
Demos = Data;
for i = 1:N
    Demos(i).OTEE_app = downsample(Data(i).OTEE_app,2);
    Demos(i).OTEE_ass = downsample(Data(i).OTEE_ass,2);
end
%}

%% DTW
%{
Demo_app = cell(1,N);
Demo_ass = cell(1,N);
for i = 1:N
    tmpN = size(Demos(i).OTEE_app,1);
%     tmpT = ((1:tmpN)*dt - dt)';
    tmpT = linspace(0,1,tmpN)';
    Demo_app{i} = [tmpT, Demos(i).OTEE_app(:,13:15)];
    tmpN = size(Demos(i).OTEE_ass,1);
%     tmpT = ((1:tmpN)*dt - dt)';
    tmpT = linspace(0,1,tmpN)';
    Demo_ass{i} = [tmpT, Demos(i).OTEE_ass(:,13:15)];
end

Demo_app_dtw = iceDTW(Demo_app,10);
Demo_ass_dtw = iceDTW(Demo_ass,10);

for i = 1:N
    Demos(i).p_app_dtw = Demo_app_dtw{i};
    Demos(i).p_ass_dtw = Demo_ass_dtw{i};
end

% Figure
figure;
labels = {'x(m)','y(m)','z(m)'};
for i = 1:N
    for j = 1:3
        subplot(3,1,j);
        plot(Demos(i).p_app_dtw(:,1), Demos(i).p_app_dtw(:,1+j));
        hold on; grid on;
        ylabel(labels{j});
    end
end
%}

%% Init. FWTPGMM

% policy_app = FWTPGMMZero(6,4,2);
% policy_ass = FWTPGMMZero(6,4,2);

%% Construct TPDemo
%{
TPDemo_app = policy_app.TPDemoConstruct(zeros(4,1),eye(4),zeros(4,1));
TPDemo_app = repmat(TPDemo_app,[1,N]);
TPDemo_ass = TPDemo_app;
for i = 1:N
    tmpA = repmat(eye(4),[1,1,2]);
    tmpb = zeros(4,2);
    % Approaching phase
    tmpSE3 = fold2SE3(Demos(i).OTEE_app(1,1:16));
    tmpA(2:4,2:4,1) = tmpSE3(1:3,1:3);
    tmpb(2:4,1) = tmpSE3(1:3,4);
    tmpSE3 = fold2SE3(Demos(i).OTEE_app(end,1:16));
    tmpA(2:4,2:4,2) = tmpSE3(1:3,1:3);
    tmpb(2:4,2) = tmpSE3(1:3,4);
    TPDemo_app(i) = policy_app.TPDemoConstruct((Demos(i).p_app_dtw)', tmpA, tmpb);
    % Assembling phase
    tmpSE3 = fold2SE3(Demos(i).OTEE_ass(1,1:16));
    tmpA(2:4,2:4,1) = tmpSE3(1:3,1:3);
    tmpb(2:4,1) = tmpSE3(1:3,4);
    tmpSE3 = fold2SE3(Demos(i).OTEE_ass(end,1:16));
    tmpA(2:4,2:4,2) = tmpSE3(1:3,1:3);
    tmpb(2:4,2) = tmpSE3(1:3,4);
    TPDemo_ass(i) = policy_ass.TPDemoConstruct((Demos(i).p_ass_dtw)', tmpA, tmpb);
    % Storation
    Demos(i).TPDemo_app = TPDemo_app(i);
    Demos(i).TPDemo_ass = TPDemo_ass(i);
end

% Figure
figure;
% App
for j = 1:2
    subplot(1,2,j);
    for i = 1:N
        tmpData = permute(Demos(i).TPDemo_app.TPData(2:4,j,:),[1,3,2]);
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color',Morandi_popsicle(j));
        hold on;
    end
    grid on; axis equal;
    xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
    view(3);
end

figure;
% Ass
for j = 1:2
    subplot(1,2,j);
    for i = 1:N
        tmpData = permute(Demos(i).TPDemo_ass.TPData(2:4,j,:),[1,3,2]);
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color',Morandi_popsicle(j));
        hold on;
    end
    grid on; axis equal;
    xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
    view(3);
end
%}

%% Learn FWTPGMM

% policy_app = policy_app.initGMMTimeBased(TPDemo_app);
% policy_app = policy_app.learnGMM(TPDemo_app);
% policy_ass = policy_ass.initGMMTimeBased(TPDemo_ass);
% policy_ass = policy_ass.learnGMM(TPDemo_ass);

%% Motion generation
%{
% Query
query_app = linspace(0,1,200);
qFrames_app = [];
qFrames_app.A = eye(4);
qFrames_app.b = zeros(4,1);
qFrames_app = repmat(qFrames_app,[1,2]);
query_ass = linspace(0,1,200);
qFrames_ass = qFrames_app;

tmpSE3 = fold2SE3(initialOTEE);
qFrames_app(1).A(2:4,2:4) = tmpSE3(1:3,1:3);
qFrames_app(1).b(2:4) = tmpSE3(1:3,4);
tmpSE3 = fold2SE3(goalOTEE);
qFrames_ass(2).A(2:4,2:4) = tmpSE3(1:3,1:3);
qFrames_ass(2).b(2:4) = tmpSE3(1:3,4);
tmpSE3 = fold2SE3(preOTEE);
qFrames_app(2).A(2:4,2:4) = tmpSE3(1:3,1:3);
qFrames_app(2).b(2:4) = tmpSE3(1:3,4);
qFrames_ass(1).A(2:4,2:4) = tmpSE3(1:3,1:3);
qFrames_ass(1).b(2:4) = tmpSE3(1:3,4);


% Setup (Just for storation)
ExpSetup = [];
ExpSetup.query_app = query_app;
ExpSetup.query_ass = query_ass;
ExpSetup.qFrames_app = qFrames_app;
ExpSetup.qFrames_ass = qFrames_ass;
%}
%{
% Approaching phase
[ExpSetup.expData_app,...
    ExpSetup.expSigma_app,...
    ExpSetup.alpha_app] = policy_app.GMR(ExpSetup.query_app,ExpSetup.qFrames_app);

% Assembling phase
[ExpSetup.expData_ass,...
    ExpSetup.expSigma_ass,...
    ExpSetup.alpha_ass] = policy_ass.GMR(ExpSetup.query_ass,ExpSetup.qFrames_ass);
%}
%{
% Figure
figure;
for i = 1:N
    tmpData = Demos(i).p_app_dtw;
    plot3(tmpData(:,2), tmpData(:,3), tmpData(:,4), 'Color', Morandi_popsicle(1));
    hold on;
end
tmpData = ExpSetup.expData_app;
plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', Morandi_popsicle(2));
grid on; axis equal; xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

% figure;
% for i = 1:N
%     plot(ExpSetup.query_app, ExpSetup.alpha_app(1,:),'Color',Morandi_popsicle(i));
%     plot(ExpSetup.query_app, ExpSetup.alpha_app(2,:),'Color',Morandi_popsicle(i));
%     hold on;
% end
% grid on;
% ylabel('\alpha');

figure;
for i = 1:N
    tmpData = Demos(i).p_ass_dtw;
    plot3(tmpData(:,2), tmpData(:,3), tmpData(:,4), 'Color', Morandi_popsicle(1));
    hold on;
end
tmpData = ExpSetup.expData_ass;
plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', Morandi_popsicle(2));
grid on; axis equal; xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);
%}

%% Robot motion generation

% writematrix(ExpSetup.expData_app','DECANTER\p_app.csv');
% writematrix(ExpSetup.expData_ass','DECANTER\p_ass.csv');
