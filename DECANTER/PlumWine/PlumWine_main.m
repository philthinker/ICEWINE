%PlumWine_main

% Haopeng Hu
% 2019.11.25
% All rights reserved

%% Data init

%{
% % UR5kg0107Data.mat

% % load('Data\20200107URData\1.mat');
% % load('Data\20200107URData\2.mat');
% 
% % We use struct arrays
% M = 11;
% demo.dataJ = [];
% demo.dataC = [];
% demo.dataCFK = [];
% demo.separation = [];
% demo = repmat(demo,[1,M]);
% 
% for i = 1:9
%     tmpJ1 = eval(strcat('j10',int2str(i)));
%     tmpJ2 = eval(strcat('j20',int2str(i)));
%     tmpN1 = size(tmpJ1,1);
%     tmpN2 = size(tmpJ2,1);
%     tmpC1 = eval(strcat('c10',int2str(i))); tmpC1 = tmpC1(end-tmpN1+1:end,:);
%     tmpC2 = eval(strcat('c20',int2str(i))); tmpC2 = tmpC2(end-tmpN2+1:end,:);
%     demo(i).dataJ = [tmpJ1;tmpJ2];
%     demo(i).dataC = [tmpC1;tmpC2];
%     demo(i).separation = [zeros(tmpN1,1);ones(tmpN2,1)];
% end
% for i = 10:11
%     tmpJ1 = eval(strcat('j1',int2str(i)));
%     tmpJ2 = eval(strcat('j2',int2str(i)));
%     tmpN1 = size(tmpJ1,1);
%     tmpN2 = size(tmpJ2,1);
%     tmpC1 = eval(strcat('c1',int2str(i))); tmpC1 = tmpC1(end-tmpN1+1:end,:);
%     tmpC2 = eval(strcat('c2',int2str(i))); tmpC2 = tmpC2(end-tmpN2+1:end,:);
%     demo(i).dataJ = [tmpJ1;tmpJ2];
%     demo(i).dataC = [tmpC1;tmpC2];
%     demo(i).separation = [zeros(tmpN1,1);ones(tmpN2,1)];
% end

% demoCFK
for i = 1:M
    demo(i).dataCFK = robot.fkine(demo(i).dataJ);
end

% Separation
demoPart1.dataJ = [];
demoPart1.dataC = [];
demoPart1.dataCFK = [];
demoPart1 = repmat(demoPart1,[1,M]);
demoPart2 = demoPart1;
for i = 1:M
    tmpIndices = demo(i).separation == 0;
    demoPart1(i).dataJ = demo(i).dataJ(tmpIndices,:);
    demoPart1(i).dataC = demo(i).dataC(tmpIndices,:);
    demoPart1(i).dataCFK = demo(i).dataCFK(:,:,tmpIndices);
    tmpIndices = demo(i).separation == 1;
    demoPart2(i).dataJ = demo(i).dataJ(tmpIndices,:);
    demoPart2(i).dataC = demo(i).dataC(tmpIndices,:);
    demoPart2(i).dataCFK = demo(i).dataCFK(:,:,tmpIndices);
end
%}
%{
% Temporal alignment (Note that there may be no need to use DTW)
% DTW in joint space
% We omit the first demo
M = 10;
tmpDataJ1 = cell(1,M);
tmpDataJ2 = cell(1,M);
for i = 1:M
    tmpDataJ1{i} = demoPart1(i).dataJ;
    tmpDataJ2{i} = demoPart2(i).dataJ;
end
tmpDataJ1dtw = iceDTW(tmpDataJ1,100);
tmpDataJ2dtw = iceDTW(tmpDataJ2,10);

demoPart1dtw.dataJ = demoPart1(1).dataJ;
demoPart1dtw = repmat(demoPart1dtw,[1,M]);
demoPart2dtw = demoPart1dtw;
for i = 1:M
    demoPart1dtw(i).dataJ = tmpDataJ1dtw{i};
    demoPart1dtw(i).dataCFK = robot.fkine(tmpDataJ1dtw{i});
    demoPart2dtw(i).dataJ = tmpDataJ2dtw{i};
    demoPart2dtw(i).dataCFK = robot.fkine(tmpDataJ2dtw{i});
end
%}

%% Robot init
% % We DO NOT use the demos in Cartesian space

%{
% Original robot demos
% It is used for figures only.
robot = UR5Zero(true);
for i = 2:M     % We omit the 1st demo
    robot = robot.addJointDemo(demo(i).dataJ);
end
robot = robot.demoFKReplace();

robot.plotJointDemo(1e-3);
robot.plotCarteDemo();
%}
%{
% Separated robot demos
robots = repmat(UR5Zero(true),[1,2]);
for i = 2:M
    robots(1) = robots(1).addJointDemo(demoPart1dtw(i).dataJ);
    robots(2) = robots(2).addJointDemo(demoPart2dtw(i).dataJ);
end
robots(1) = robots(1).demoFKReplace();
robots(2) = robots(2).demoFKReplace();

robots(1).plotJointDemo();
robots(2).plotJointDemo();
robots(1).plotCarteDemo();
robots(2).plotCarteDemo();
%}

%% Plot demonstration

%{
% % Original demo
% figure;
% for i = 1:M
%     tmpData = robot.demoCartesian{i};
%     tmpData = permute(tmpData(1:3,4,:),[1,3,2]);
%     plot3(tmpData(1,:),tmpData(2,:),tmpData(3,:),'Color',[143/255,130/255,188/255],'LineWidth',3);
%     hold on;
% end
% axis equal;

% % Cutted demo
% figure;
% for i = 1:9
%     tmpData = robots(1).demoCartesian{i};
%     tmpData = permute(tmpData(1:3,4,:),[1,3,2]);
%     plot3(tmpData(1,:),tmpData(2,:),tmpData(3,:),'Color',[235/255,104/255,119/255],'LineWidth',3);
%     hold on;
%     tmpData = robots(2).demoCartesian{i};
%     tmpData = permute(tmpData(1:3,4,:),[1,3,2]);
%     plot3(tmpData(1,:),tmpData(2,:),tmpData(3,:),'Color',[0/255,160/255,233/255],'LineWidth',3);
% end
% axis equal;
%}

%% GMM in Part 1 and DMP in Part 2

%{
% Part 1: GMM
model1 = GMMOne(5,7);
model1 = model1.initGMMTimeBased(DataJ1);
model1 = model1.learnGMM(DataJ1);

query_model1 = 1e-3*(0:size(demoPart1dtw(1).dataJ,1)-1);

[expData_model1,expSigma_model1] = model1.GMR(query_model1);

robots(1).exeJoint = expData_model1';
robots(1).plotJointDemoPlus(1e-3,[query_model1',expData_model1']);
robots(1).exeCartesian = robots(1).fkine(expData_model1');
robots(1).plotCarteDemo(true);
%}

%{
% Part 2: DMP (ProMP)

% DataJ2dyna.data = [];
% DataJ2dyna = repmat(DataJ2dyna,[1,M]);
% for i = 1:M
%     DataJ2dyna(i).data = demoPart2dtw(i).dataJ';
% end

model2 = ProMPZero(6,20,1e-4);
model2 = model2.leanLRR(DataJ2dyna);

expData_model2 = model2.reproduct(50);

robots(2).exeJoint = expData_model2';
robots(2).plotJointDemoPlus(1e-3,[1e-3*(0:size(expData_model2,2)-1)',expData_model2']);
robots(2).exeCartesian = robots(2).fkine(expData_model2');
robots(2).plotCarteDemo(true);
%}



%% Dual ProMPs



%% Simple GMM for contrast

%{
% Data prepare
DataJ1 = robots(1).getDemoJoint(1e-3,0);
DataJ2 = robots(2).getDemoJoint(1e-3,max(DataJ1(:,1)));
% DataJ = [DataJ1;DataJ2]';
%}
%{
queryGMMCon = linspace(min(DataJ(1,:)),max(DataJ(1,:)),100);

gmmContrast = GMMOne(10,7);
gmmContrast = gmmContrast.initGMMTimeBased(DataJ);
gmmContrast = gmmContrast.learnGMM(DataJ);

[expDataGMMCon,expSigmaGMMCon] = gmmContrast.GMR(queryGMMCon);

robotGMMCon = robot;
robotGMMCon.plotJointDemoPlus(1e-3,[queryGMMCon', expDataGMMCon']);
robotGMMCon.exeCartesian = robotGMMCon.fkine(expDataGMMCon');
robotGMMCon.plotCarteDemo(true);
%}

% expDataGMMConL = gmmContrast.GMR(linspace(min(queryGMMCon),max(queryGMMCon),500));

%% Simple GP for contrast

%{
queryGPCon = linspace(min(DataJ(1,:)),max(DataJ(1,:)),100);

gpContrast = GPZero(DataJ);
gpContrast = gpContrast.setParam(1e0,1e-4,1e-6);
gpContrast = gpContrast.preGPR();

expDataGPCon = gpContrast.GPR(queryGPCon);

robotGPCon = robot;
robotGPCon.plotJointDemoPlus(1e-3,[expDataGPCon.Data']);
robotGPCon.exeCartesian = robotGPCon.fkine((expDataGPCon.Data(2:end,:))');
robotGPCon.plotCarteDemo(true);
%}

%% Simple DMP for contrast
% We use ProMP instead of standard DMP
% Note that when you use dynamic models, the time series are not trivial.

%{
% DataJdyna.data = DataJ(2:end,:);    % Trivial
% DataJdyna = repmat(DataJdyna,[1,M]);
% for i = 1:M
%     DataJdyna(i).data = [(demoPart1dtw(i).dataJ)',(demoPart2dtw(i).dataJ)'];
% end
%}
%{
dmpContrast = ProMPZero(6,20);
dmpContrast = dmpContrast.leanLRR(DataJdyna);

expDataDMP = dmpContrast.reproduct(100);

robotDMPCon = robot;
robotDMPCon.plotJointDemoPlus(1e-3,[1e-3*(0:99)',expDataDMP']);
robotDMPCon.exeCartesian = robotDMPCon.fkine(expDataDMP');
robotDMPCon.plotCarteDemo(true);
%}

%% File generation

% totxt(expDataDMPCon',5,4,'conDMP');
% totxt(expDataGMMCon',5,4,'conGMM');
% totxt(expDataGPCon.Data(2:end,:)',5,4,'conGP');
% 
% expDataDMPConC = robotDMPCon.toMoveitForm();
% expDataGMMConC = robotGMMCon.toMoveitForm();
% expDataGPConC = robotGPCon.toMoveitForm();
% 
% totxt(expDataDMPConC,5,4,'comDMPC');
% totxt(expDataGMMConC,5,4,'conGMMC');
% totxt(expDataGMMConC,5,4,'conGPC');
