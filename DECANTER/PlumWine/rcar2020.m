%rcar2020

% Haopeng Hu
% 2020.02.24
% All rights reserved

%% load data
% Load data from the PlumWine project, or figure in 'Figure' folder
% load('Data\ur5kg0110.mat') for raw data;
% load('Data\ur5kgRCAR2020.mat') for simple data

M = 9;
dt = 0.01;

%% TP-GMM for part I
%{
% % Here we use demoPart1dtw as demo data
% % Init. data for TP-GMM learning and reproduction
% gmmPartI = TPGMM(6,4,2);
% demoPart1TP = cell(1,M);
% A = repmat(eye(4),[1,1,2]);
% b = zeros(4,2);
% for m = 1:M
%     A(2:4,2:4,1) = demoPart1dtw(m).dataCFK(1:3,1:3,1);
%     A(2:4,2:4,end) = demoPart1dtw(m).dataCFK(1:3,1:3,end);
%     b(2:end,1) = demoPart1dtw(m).dataCFK(1:3,4,1);
%     b(2:end,end) = demoPart1dtw(m).dataCFK(1:3,4,end);
%     tmpData = demoPart1dtw(m).dataCFK(1:3,4,:);
%     tmpData = permute(tmpData,[1,3,2]);
%     tmpData = tmpData'; tmpDemo = [linspace(0,1,size(tmpData,1))',tmpData];
%     demoPart1TP{m} = gmmPartI.dataReconstruct(A,b,tmpDemo);
% end

% % Learn a TP-GMM
% gmmPartI = gmmPartI.initGMMTimeBased(demoPart1TP);
% gmmPartI = gmmPartI.learnGMM(demoPart1TP);

% % GMR
% query = linspace(0,1,1000);
% queryFrames = [];
% queryFrames.A = demoPart1TP{1}.A(:,:,1); queryFrames.b = demoPart1TP{1}.b(:,1);
% queryFrames = repmat(queryFrames,[1,2]);
% queryFrames(2).A = demoPart1TP{1}.A(:,:,2); queryFrames(2).b = demoPart1TP{2}.b(:,2);
% expPPartI = gmmPartI.GMR(query,queryFrames);

% % Figure
% robotPartI = UR5Zero(true);
% robotPartI.exeCartesian = p2SE3(expPPartI,eye(3));
% robotPartI.plotCarte(0);
%}
%% DMP for part II
%{
% % Here we use demoPart2
% % Init. data for DMP learning and reproduction
% dmpsPartII = repmat(IjspeertDMPOne(20),[6,M]);
% dmpPartII = repmat(IjspeertDMPOne(20),[6,1]);
% % Learn DMPs for each demo
% for m = 1:M
%     for i = 1:6
%         tmpJ = demoPart2(m).dataJ(:,i);
%         tmpD = computeDynaData(tmpJ,dt);
%         dmpsPartII(i,m) = dmpsPartII(i,m).learnLWR(tmpD);
%     end
% end

% % Maximum-likelihood
% for i = 1:6
%     tmpw = zeros(20,1);
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

% % Figure
% robotPartII = UR5Zero(true);
% robotPartII.exeJoint = expJPartII;
% robotPartII.plotJoint(0);
% robotPartII.exeCartesian = robotPartII.fkine(expJPartII);
% robotPartII.plotCarte(0);
%}
%% Merge Part I & part II

% robotGPCon.plotCarte(0);

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
%% TP-GMM comparison
%{
% % Init. data
% M = 9;
% gmmCom = TPGMM(8,4,2);
% A = repmat(eye(4),[1,1,2]);
% b = zeros(4,2);
% demoTP = cell(1,M);
% for m = 1:M
%     A(2:4,2:4,1) = demoDTW(m).dataCFK(1:3,1:3,1);
%     A(2:4,2:4,end) = demoDTW(m).dataCFK(1:3,1:3,end);
%     b(2:end,1) = demoDTW(m).dataCFK(1:3,4,1);
%     b(2:end,end) = demoDTW(m).dataCFK(1:3,4,end);
%     tmpData = demoDTW(m).dataCFK(1:3,4,:);
%     tmpData = permute(tmpData,[1,3,2]);
%     tmpData = tmpData'; tmpDemo = [linspace(0,1,size(tmpData,1))',tmpData];
%     demoTP{m} = gmmCom.dataReconstruct(A,b,tmpDemo);
% end

% % Learn a TP-GMM
% gmmCom = gmmCom.initGMMTimeBased(demoTP);
% gmmCom = gmmCom.learnGMM(demoTP);

% % GMR
% query = linspace(0,1,1000);
% queryFrames = [];
% queryFrames.A = demoTP{1}.A(:,:,1); queryFrames.b = demoTP{1}.b(:,1);
% queryFrames = repmat(queryFrames,[1,2]);
% queryFrames(2).A = demoTP{1}.A(:,:,2); queryFrames(2).b = demoTP{2}.b(:,2);
% expTPGMMCom = gmmCom.GMR(query,queryFrames);

% % Figure
% robotGMMCom = UR5Zero(true);
% robotGMMCom.exeCartesian = p2SE3(expTPGMMCom,eye(3));
% robotGMMCom.plotCarte(0);
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