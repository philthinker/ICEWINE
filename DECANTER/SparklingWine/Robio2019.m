% Robio2019

% Haopeng Hu
% 2019.09.25
% All rights reserved

%% Load demonstration data
% % If the data is raw, please run Robio2019Data in advance.

% load('fruits\robio2019.mat');

%% Temporal alignment

% demoJointDTW = iceDTW(demoJoint,100);

%% Init panda by PandaZero

% panda = PandaZero();

%% Add demonstration data to panda

% for i = 1:9
%     panda = panda.addJointDemo(demoJointDTW{i});
%     panda = panda.addCartesianDemo(demoPose{i});
% end

%% Demonstration data show

% for i = 1:6
%     panda.plotJoint(i);
% end
% for i = 1:6
%     panda.plotCarte(i);
% end
% panda.plotCarteDemo();
panda.plotJointDemo();



