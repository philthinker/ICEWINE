%rcar2020

% Haopeng Hu
% 2020.02.24
% All rights reserved

%% load data
% Load data from the PlumWine project, or figure in 'Figure' folder
% load('Data\ur5kg0110.mat') for raw data;
% load('Data\ur5kgRCAR2020.mat') for simple data
%{
M = 9;
dt = 0.01;
ABeginning = eye(4); 
AHalf = eye(4); AEnd = eye(4);
bBeginning = zeros(4,1); bHalf = zeros(4,1); bEnd = zeros(4,1);

% ABeginning(2:4,2:4) = demo(1).dataCFK(1:3,1:3,1);
AHalf(2:4,2:4) = demoPart1dtw(1).dataCFK(1:3,1:3,end);
ABeginning = AHalf;
AEnd(2:4,2:4) = demo(1).dataCFK(1:3,1:3,end);
bBeginning(2:4) = demo(1).dataCFK(1:3,4,1);
% bBeginning(2:4) = bBeginning(2:4);
bHalf(2:4) = demoPart1dtw(1).dataCFK(1:3,4,end);
bEnd(2:4) = demo(1).dataCFK(1:3,4,end);
%}
%% TP-GMM for part I
%{
% Here we use demoPart1dtw as demo data
% Init. data for TP-GMM learning and reproduction
gmmPartI = TPGMM(6,4,2);
demoPart1TP = cell(1,M);
A = repmat(eye(4),[1,1,2]);
b = zeros(4,2);
for m = 1:M
    A(2:4,2:4,1) = demoPart1dtw(m).dataCFK(1:3,1:3,1);
    A(2:4,2:4,end) = demoPart1dtw(m).dataCFK(1:3,1:3,end);
    b(2:end,1) = demoPart1dtw(m).dataCFK(1:3,4,1);
    b(2:end,end) = demoPart1dtw(m).dataCFK(1:3,4,end);
    tmpData = demoPart1dtw(m).dataCFK(1:3,4,:);
    tmpData = permute(tmpData,[1,3,2]);
    tmpData = tmpData'; tmpDemo = [linspace(0,1,size(tmpData,1))',tmpData];
    demoPart1TP{m} = gmmPartI.dataReconstruct(A,b,tmpDemo);
end

% Learn a TP-GMM
gmmPartI = gmmPartI.initGMMTimeBased(demoPart1TP);
gmmPartI = gmmPartI.learnGMM(demoPart1TP);

% GMR
query = linspace(0,1,1000);
queryFrames = [];
% queryFrames.A = demoPart1TP{1}.A(:,:,1); queryFrames.b = demoPart1TP{1}.b(:,1);
queryFrames.A = ABeginning; queryFrames.b = bBeginning;
queryFrames = repmat(queryFrames,[1,2]);
% queryFrames(2).A = demoPart1TP{1}.A(:,:,2); queryFrames(2).b = demoPart1TP{2}.b(:,2);
queryFrames(2).A = AHalf; queryFrames(2).b = bHalf;
expPPartI = gmmPartI.GMR(query,queryFrames);

% Figure
robotPartI = UR5Zero(true);
robotPartI.exeCartesian = p2SE3(expPPartI,eye(3));
robotPartI.plotCarte(0);
%}
%% Policy I Figure
%{
% % % Here we use GMMOne for drawing figures
% % Data used is demoPart1dtw
% % Data init.
% demoPart1dtwA1 = demoPart1dtw; demoPart1dtwA2 = demoPart1dtw;
% for m = 1:M
%     A = repmat(eye(3),[1,1,2]);
%     A(1:3,1:3,1) = demoPart1dtw(m).dataCFK(1:3,1:3,1);
%     A(1:3,1:3,2) = demoPart1dtw(m).dataCFK(1:3,1:3,end);
%     b = zeros(3,2);
%     b(1:3,1) = demoPart1dtw(m).dataCFK(1:3,4,1);
%     b(1:3,2) = demoPart1dtw(m).dataCFK(1:3,4,end);
%     T = repmat(eye(4),[1,1,2]);
%     T(:,:,1) = SO3P2SE3(A(:,:,1),b(:,1));
%     T(:,:,2) = SO3P2SE3(A(:,:,2),b(:,2));
%     N = size(demoPart1dtw(m).dataCFK,3);
%     for i = 1:N
%         demoPart1dtwA1(m).dataCFK(:,:,i) = T(:,:,1)\demoPart1dtwA1(m).dataCFK(:,:,i);
%         demoPart1dtwA2(m).dataCFK(:,:,i) = T(:,:,2)\demoPart1dtwA2(m).dataCFK(:,:,i);
%     end
% end

% % Init. GMM
% gmmFigure1 = GMMOne(6,4);
% gmmFigure2 = gmmFigure1;
% dataPPart1dtwA1 = se3DataRegulate(demoPart1dtwA1,1);
% dataPPart1dtwA2 = se3DataRegulate(demoPart1dtwA2,1);
% % gmmFigure1 = gmmFigure1.initGMMKMeans(dataPPart1dtwA1);
% % gmmFigure2 = gmmFigure2.initGMMKMeans(dataPPart1dtwA2);
% gmmFigure1 = gmmFigure1.initGMMTimeBased(dataPPart1dtwA1);
% gmmFigure2 = gmmFigure2.initGMMTimeBased(dataPPart1dtwA2);
% gmmFigure1 = gmmFigure1.learnGMM(dataPPart1dtwA1);
% gmmFigure2 = gmmFigure2.learnGMM(dataPPart1dtwA2);

% % Figure
% figure; hold on;% Figure 1
% % plot demos
% for m = 1:M
%     plot3(permute(demoPart1dtwA1(m).dataCFK(1,4,:),[3,1,2]),...
%         permute(demoPart1dtwA1(m).dataCFK(2,4,:),[3,1,2]),...
%         permute(demoPart1dtwA1(m).dataCFK(3,4,:),[3,1,2]),'Color',[0.4,0.4,0.4],'LineWidth',1);
% end
% grid on; axis equal;
% % plot GMM
% plotGMM3SC(gmmFigure1.Mu(2:4,:),gmmFigure1.Sigma(2:4,2:4,:),[1,0.5,0],0.6);
% 
% figure; hold on;% Figure 2
% % plot demos
% for m = 1:M
%     plot3(permute(demoPart1dtwA2(m).dataCFK(1,4,:),[3,1,2]),...
%         permute(demoPart1dtwA2(m).dataCFK(2,4,:),[3,1,2]),...
%         permute(demoPart1dtwA2(m).dataCFK(3,4,:),[3,1,2]),'Color',[0.4,0.4,0.4],'LineWidth',1);
% end
% grid on; axis equal;
% % plot GMM
% plotGMM3SC(gmmFigure2.Mu(2:4,:),gmmFigure2.Sigma(2:4,2:4,:),[0,0.5,1],0.6);
%}
%% DMP for part II
%{
% % Here we use demoPart2
% % Init. data for DMP learning and reproduction
% dmpsPartII = repmat(IjspeertDMPOne(40),[6,M]);
% dmpPartII = repmat(IjspeertDMPOne(40),[6,1]);
% % Learn DMPs for each demo
% for m = 1:M
%     for i = 1:6
%         tmpJ = demoPart2(m).dataJ(:,i);
%         tmpD = computeDynaData(tmpJ,dt);
%         dmpsPartII(i,m) = dmpsPartII(i,m).learnLWR(tmpD);
%     end
% end
% 
% % Maximum-likelihood
% for i = 1:6
%     tmpw = zeros(40,1);
%     for m = 1:M
%         tmpw = tmpw + dmpsPartII(i,m).w;
%     end
%     tmpw = tmpw./M;
%     dmpPartII(i).w = tmpw;
% end

% % Run
% % Init. y0 and g
% queryY0 = demoPart2(1).dataJ(1,:);
% queryG = demoPart2(1).dataJ(end,:);
% N = 1000;
% expJDynaPartII = zeros(N,3,6);
% expJPartII = zeros(N,6);
% for i = 1:6
%     expJDynaPartII(:,:,i) = dmpPartII(i).run(queryY0(i),queryG(i),N);
%     expJPartII(:,i) = expJDynaPartII(:,1,i);
% end
% 
% % % Figure
% robotPartII = UR5Zero(true);
% robotPartII.exeJoint = expJPartII;
% robotPartII.plotJoint(0);
% for m = 1:M
%     robotPartII = robotPartII.addJointDemo(demoPart2(m).dataJ);
% end
% robotPartII.exeCartesian = robotPartII.fkine(expJPartII);
% robotPartII.plotCarte(0);
%}
%% Policy II Figure
%{
% Here we use ProMP
for m = 1:M
    demoPart2(m).data = demoPart2(m).dataJ';
end
promp = ProMPZero(6,40);
promp = promp.learnLRR(demoPart2);

% Run
N = 1000;
expJPartII = promp.reproduct(N);

% Figure
figure;
% Plot demos
for i = 1:6
    subplot(6,1,i);
    for m = 1:M
        t = linspace(0,1,size(demoPart2(m).data,2));
        plot(t,demoPart2(m).data(i,:),'Color',[0.6, 0.6, 0.6],'LineWidth',1);
%         aa = axis;
%         axis([0,1,min(demoPart2(m).data(i,:))-0.001,max(demoPart2(m).data(i,:))+0.001]);
        hold on;
    end
    t = linspace(0,1,N);
    plot(t,expJPartII(i,:),'Color',[0,0.5,1],'LineWidth',1);
    grid on; aa = axis;
    axis([0,1,aa(3)-0.01,aa(4)+0.01]);
    ylabel(strcat('Joint ',int2str(i)));
end

% Robot
robotPartII = UR5Zero(true);
robotPartII.exeJoint = expJPartII';
robotPartII.exeCartesian = robotPartII.fkine(robotPartII.exeJoint);
robotPartII.plotJoint(0);
robotPartII.plotCarte(0);
%}
%% DMP comparison
%{
% % Init.
% M = 9; dt = 0.01; K = 50;
% dmpsCom = repmat(IjspeertDMPOne(K),[6,M]);
% % Learn
% for m = 1:M
%     for i = 1:6 % the 6-th the always zero
%         % learn each DMP
%         tmpJ = demo(m).dataJ(:,i);
%         tmpD = computeDynaData(tmpJ,dt);
%         dmpsCom(i,m) = dmpsCom(i,m).learnLWR(tmpD);
%     end
% end
% dmpCom = repmat(IjspeertDMPOne(K),[6,1]);
% % Maximum-likelihood
% for i = 1:6
%     tmpw = zeros(K,1);
%     for m = 1:M
%         tmpw = tmpw + dmpsCom(i,m).w;
%     end
%     dmpCom(i).w = tmpw./M;
% end

% % Run
% % Init. y0 and g
% tmpy0 = zeros(M,6); tmpg = zeros(M,6);
% y0 = zeros(1,6); g = zeros(1,6);
% N = 1000; tmpY = zeros(N,3,6); dmpComY = zeros(N,6);
% for i = 1:6
%     for m = 1:M
%         tmpy0(m,i) = demo(m).dataJ(1,i);
%         tmpg(m,i) = demo(m).dataJ(end,i);
%     end
%     y0(i) = mean(tmpy0(:,i));
%     g(i) = mean(tmpg(:,i));
%     tmpY(:,:,i) = dmpCom(i).run(y0(i),g(i),N);
%     dmpComY(:,i) = tmpY(:,1,i);
% end

% % Figure
% robotDMPCom = UR5Zero(true);
% robotDMPCom.exeJoint = dmpComY;
% robotDMPCom.plotJoint(0);
% robotDMPCom.exeCartesian = robotDMPCom.fkine(dmpComY);
% robotDMPCom.plotCarte(0);
%}
%% ProMP comparison
%{
% Init.
for m = 1:M
    demo(m).data = demo(m).dataJ';
end
prompCom = ProMPZero(6,40);
prompCom = prompCom.learnLRR(demo);

% Run
N = 1000;
expProMPCom = prompCom.reproduct(N);

% Figure
figure;
% Plot demos
for i = 1:6
    subplot(6,1,i);
    for m = 1:M
        t = linspace(0,1,size(demo(m).data,2));
        plot(t,demo(m).data(i,:),'Color',[0.6, 0.6, 0.6],'LineWidth',1);
        hold on;
    end
    t = linspace(0,1,N);
    plot(t,expProMPCom(i,:),'Color',[0,0.5,1],'LineWidth',1);
    grid on; aa = axis;
    axis([0,1,aa(3)-0.01,aa(4)+0.01]);
    ylabel(strcat('Joint ',int2str(i)));
end

% Robot
robotProMPCom = UR5Zero(true);
robotProMPCom.exeJoint = expProMPCom';
robotProMPCom.exeCartesian = robotProMPCom.fkine(robotProMPCom.exeJoint);
robotProMPCom.plotJoint(0);
robotProMPCom.plotCarte(0);

% Run with modulation
y0Point.data = (demo(1).dataJ(1,:))';
% y0Point.data = y0Point.data + [0.1,0,0,0,0,0]';
y0Point.t = 0;
y0Point.Sigma = zeros(6,6,1);
prompComM = prompCom.modulate(y0Point);
expProMPComM = prompComM.reproduct(N);

% Robot
robotProMPComM = UR5Zero(true);
robotProMPComM.exeJoint = expProMPComM';
robotProMPComM.exeCartesian = robotProMPComM.fkine(robotProMPComM.exeJoint);
robotProMPComM.plotJoint(0);
robotProMPComM.plotCarte(0);
%}
%% TP-GMM comparison
%{
% Init. data
M = 9;
gmmCom = TPGMM(6,4,2);
A = repmat(eye(4),[1,1,2]);
b = zeros(4,2);
demoTP = cell(1,M);
for m = 1:M
    A(2:4,2:4,1) = demoDTW(m).dataCFK(1:3,1:3,1);
    A(2:4,2:4,end) = demoDTW(m).dataCFK(1:3,1:3,end);
    b(2:end,1) = demoDTW(m).dataCFK(1:3,4,1);
    b(2:end,end) = demoDTW(m).dataCFK(1:3,4,end);
    tmpData = demoDTW(m).dataCFK(1:3,4,:);
    tmpData = permute(tmpData,[1,3,2]);
    tmpData = tmpData'; tmpDemo = [linspace(0,1,size(tmpData,1))',tmpData];
    demoTP{m} = gmmCom.dataReconstruct(A,b,tmpDemo);
end

% Learn a TP-GMM
gmmCom = gmmCom.initGMMTimeBased(demoTP);
gmmCom = gmmCom.learnGMM(demoTP);

% GMR
query = linspace(0,1,1000);
queryFrames = [];
% queryFrames.A = demoTP{1}.A(:,:,1); queryFrames.b = demoTP{1}.b(:,1);
queryFrames.A = ABeginning; queryFrames.b = bBeginning;
queryFrames = repmat(queryFrames,[1,2]);
% queryFrames(2).A = demoTP{1}.A(:,:,2); queryFrames(2).b = demoTP{2}.b(:,2);
queryFrames(2).A = AHalf; queryFrames(2).b = bHalf;
expTPGMMCom = gmmCom.GMR(query,queryFrames);

% Figure
robotGMMCom = UR5Zero(true);
robotGMMCom.exeCartesian = p2SE3(expTPGMMCom,eye(3));
robotGMMCom.plotCarte(0);
%}
%% Comparison figure
%{
% What we need are: demos, merged pbd traj., DMP/ProMP traj., TP-GMM traj..
% TP-GMM traj.
tpgmmTraj = robotGMMCom.exeCartesian;
% DMP/ProMP traj.
prompMTraj = robotProMPComM.exeCartesian;
% merged PbD traj.
pbdTrajPartI = robotPartI.exeCartesian;
pbdTrajPartII = robotPartII.exeCartesian;
pbdTraj = zeros(4,4,2*N);
pbdTraj(:,:,1:N) = pbdTrajPartI;
pbdTraj(:,:,N+1:end) = pbdTrajPartII;
% small adjustment
tpgmmTrajM = tpgmmTraj;
tpgmmTrajM(1:3,4,:) = tpgmmTraj(1:3,4,:) + prompMTraj(1:3,4,1) - tpgmmTraj(1:3,4,1);
pbdTrajM = zeros(4,4,2*N+1); pbdTrajM(:,:,2:end) = pbdTraj;  pbdTrajM(:,:,1) = prompMTraj(:,:,1);
%}
%{
tmpP = zeros(3,100); tmpP(:,1) = tpgmmTrajM(1:3,4,end); tmpP(:,end) = bEnd(2:end);
for i = 1:3
    tmpP(i,1:end) = pchip([0,1],[tmpP(i,1),tmpP(i,end)],linspace(0,1,100));
end
tmpVia = zeros(4,10); tmpVia(2:4,1) = tpgmmTrajM(1:3,4,end); tmpVia(1,1) = 0;
tmpVia(2:4,end) = bEnd(2:end); tmpVia(1,end) = 1; tmpL = 10;
for i = 2:9
    tmpVia(1,i) = i*tmpL/100;
    tmpVia(2:end,i) = tmpP(:,(i-1)*tmpL);
    if i < 6
        tmpVia(2:end,i) = tmpVia(2:end,i) + [0;-0.0006*i;0];
    else
        tmpVia(2:end,i) = tmpVia(2:end,i) + [0;-0.0006*(10-i);0];
    end
end
for i = 1:3
    tmpP(i,1:end) = spline(tmpVia(1,:),tmpVia(i+1,:),linspace(0,1,100));
end

tpgmmTrajM2 = zeros(4,4,1100); tpgmmTrajM2(:,:,1:1000) = tpgmmTrajM; tpgmmTrajM2(1:3,4,1001:end) = tmpP;
%}
%{
% Figure
figure; hold on;
% plot demos
for m = 1:M
    plot3(permute(demo(m).dataCFK(1,4,:),[3,1,2]),...
        permute(demo(m).dataCFK(2,4,:),[3,1,2]),...
        permute(demo(m).dataCFK(3,4,:),[3,1,2]),'Color',[0.4,0.4,0.4],'LineWidth',0.8);
end
% plot TP-GMM traj.
plot3(permute(tpgmmTrajM2(1,4,:),[3,1,2]),...
    permute(tpgmmTrajM2(2,4,:),[3,1,2]),...
    permute(tpgmmTrajM2(3,4,:),[3,1,2]),'Color',[1,0.5,0],'LineWidth',1.5);
% plot modulated ProMP traj.
plot3(permute(prompMTraj(1,4,:),[3,1,2]),...
    permute(prompMTraj(2,4,:),[3,1,2]),...
    permute(prompMTraj(3,4,:),[3,1,2]),'Color',[0,0.5,1],'LineWidth',1.5);
% plot Pbd traj.
plot3(permute(pbdTrajM(1,4,:),[3,1,2]),...
    permute(pbdTrajM(2,4,:),[3,1,2]),...
    permute(pbdTrajM(3,4,:),[3,1,2]),'Color',[0,1,0],'LineWidth',2);
grid on; axis equal;
% View
view(3);
xlabel('x'); ylabel('y'); zlabel('z');
aa = axis;
axis([aa(1:2),0.1,0.25,-0.1,0.8]);
%}
%% Auxiliary Func.

function [D] = computeDynaData(y,dt)
    %   Compute dynamic data
    %   y: N x 1
    %   dt: scalar
    D = repmat(y,[1,3]);
    D(1,2) = 0; D(2:end,2) = (y(2:end) - y(1:end-1))/dt;
    D(1,3) = 0; D(2:end,3) = (D(2:end,2) - D(1:end-1,2))/dt;
    for i = 1:size(D,1)
        if D(i,2) == 0
            D(i,2) = 1e-6;
        end
        if D(i,3) == 0
            D(i,3) = 1e-6;
        end
    end
end

function [SE3Data] = p2SE3(pData,A)
    %   Transsform positions to SE3 form with orientation A
    %   pData: 3 x N, positions
    %   A: 3 x 3, SO3
    N = size(pData,2);
    SE3Data = repmat(eye(4),[1,1,N]);
    for i = 1:N
        SE3Data(1:3,1:3,i) = A;
        SE3Data(1:3,4,i) = pData(:,i);
    end
end

function [Data] = se3DataRegulate(demos,mode)
    %   demos: (dataCFK: 3 x 3 x N) x M
    %   mode: 1 for GMR usage
    %   Data: 3 x (M*N) or 4 x (M*N)
    M = length(demos);
    N = size(demos(1).dataCFK,3);
    if mode == 1
        % GMR usage, one more dimension
        Data = zeros(4,M*N);
        n = 1;
        for i = 1:M
            for j = 1:N
                Data(1,n) = j/N;
                Data(2:4,n) = demos(i).dataCFK(1:3,4,j);
                n = n + 1;
            end
        end
    else
        Data = zeros(3,M*N);
        n = 1;
        for i = 1:M
            for j = 1:N
                Data(:,n) = demos(i).dataCFK(1:3,4,j);
                n = n + 1;
            end
        end
    end
end