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
%}
%{
% demoCFK
for i = 1:M
    demo(i).dataCFK = robot.fkine(demo(i).dataJ);
end
%}

%% Robot init
% % We DO NOT use the demos in Cartesian space

%{ 
% Original robot demo
robot = UR5Zero(true);
for i = 1:M
    robot = robot.addJointDemo(demo(i).dataJ);
end
robot = robot.demoFKReplace();

robot.plotJointDemo();
robot.plotCarteDemo();
%}




