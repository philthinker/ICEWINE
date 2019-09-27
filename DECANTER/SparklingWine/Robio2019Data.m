%Robio2019Data

% Haopeng Hu
% 2019.09.27
% All rights reserved

%% Load demos
% % load('20190927PandaKinestheticAssembly_raw.mat') for the raw

demoJoint = {D1joint, D2joint, D3joint, D4joint, D5joint, D6joint,  D8joint, D9joint, D10joint};
demoPose = {D1pose, D2pose, D3pose, D4pose, D5pose, D6pose,  D8pose, D9pose, D10pose};

%% We do not need the demo of the 3th joint

M = 9;
joint3 = zeros(1,M);
for i = 1:M
    joint3(i) = demoJoint{i}(end,3);
end
for i = 1:M
%     demoJoint{i}(:,3) = repmat( mean(demoJoint{i}(:,3)), size(demoJoint{i}(:,3)) );
    demoJoint{i}(:,3) = repmat( mean(joint3), size(demoJoint{i}(:,3)) );
end