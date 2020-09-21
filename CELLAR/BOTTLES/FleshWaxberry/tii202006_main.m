%tii202006_main
%   Just generate what we want for figure and exp
%   No more temporary codes and data
%
%   Haopeng Hu
%   2020.08.06 Good Luck
%   All rights reserved

% load('Data\lead0806_test.mat')

% % Polices have already been trained

%% For demo-retrieve pre
% What we want are
%   the initial and final joint position,
%   the position and quat trajectory in pre phase and cis phase
%   the K trajectory in pre phase and cis phase
%   the dummy poses for compensation
% What we figure are
%   the quat trajectory
%   the wrench trajectory

% % 'Data_pre_w'
% % 'Data_pre_q'
%{
% Retrieve quat
for i= 1:M
    Data_pre_w(i).tpData2 = (Data_pre_w(i).query_frame(2).A(2:4,2:4))'*(Data_pre_w(i).expData_p - Data_pre_w(i).query_frame(2).b(2:4,1));
    Data_pre_q(i).query_q = Data_pre_w(i).tpData2(3,:);
    [Data_pre_q(i).expData_eta,Data_pre_q(i).expSigma_eta] = ...
        policy_pre_quat.GMR(Data_pre_q(i).query_q);
    Data_pre_q(i).expData_q = policy_pre_quat.expmap(Data_pre_q(i).expData_eta);
    N = size(Data_pre_q(i).query_q,2);
    Data_pre_q(i).expData_realq = quatProduct( Data_pre_q(i).expData_q, repmat(Data_pre_q(i).qa,[1,N]));
    Data_pre_q(i).expData_realqPlus = [Data_pre_q(i).expData_realq,Data_pre_q(i).qa];
end
%}
%{
for i = 1:M
    writematrix(Data_pre_w(i).expData_pPlus',strcat('DECANTER\FleshWaxberry\Data\08-06\ret\pre\dataprep',int2str(i),'.csv'));
    writematrix(quatRegulate(Data_pre_q(i).expData_realqPlus)',strcat('DECANTER\FleshWaxberry\Data\08-06\ret\pre\datapreq',int2str(i),'.csv'));
    writematrix([Demo(i).pre.JP(1,:);Demo(i).pre.JP(end,:)],strcat('DECANTER\FleshWaxberry\Data\08-06\ret\pre\dataprej',int2str(i),'.csv'));
end
%}
%{
% The real q data
for i = 1:M
    tmp_log =...
        readmatrix(strcat('DECANTER\FleshWaxberry\Data\08-06\ret\pre\logpreq',int2str(i),'.csv'));
    % Get rid of the trivial data
    tmp_log_SE3 = fold2SE3(tmp_log(:,1:16));
    Data_pre_q(i).exeData_realq = tform2quat(tmp_log_SE3)';
    tmp_log_SO3 = tmp_log_SE3(1:3,1:3,:);
    for j = 1:size(tmp_log_SO3,3)
        tmp_log_SO3(:,:,j) = (Demos_pre(i).R(:,:,2))' * tmp_log_SE3(1:3,1:3,j);
    end
    Data_pre_q(i).exeData_q = rotm2quat(tmp_log_SO3)';
end
%}
% % Generate K
%{
N = size(Data_pre_w(1).expData_pPlus,2);
K = repmat(linspace(300,400,N)',[1,6]);
writematrix(K,'DECANTER\FleshWaxberry\Data\08-06\ret\pre\dataprek.csv');
%}

%% For gen pre

%% For demo-retrieve cis
% What we want are
%   the initial and final joint position,
%   the position and quat trajectory in cis phase
%   the K trajectory in cis phase
% What we figure are
%   the quat trajectory
%   the wrench trajectory

% Generate K
%{
stiffness = 30;
N = 500;
K = 30 * ones(N,6);
K(:,4:6) = sqrt(K(:,4:6));
%}
% File out
%{
tmp_file = 'DECANTER\FleshWaxberry\Data\08-06\ret\cis\datacis';
for i = 1:M
    % Write the init. and final joint position for compensation
    writematrix([Demo(i).cis.JP(1,:); Demo(i).cis.JP(end,:)],...
        strcat(tmp_file, 'j', int2str(i), '.csv'));
    % Write position data
    writematrix(Data_cis(i).expData_p'/1000,...
        strcat(tmp_file, 'p', int2str(i), '.csv'));
    % Write quaternion data
    writematrix(Data_cis(i).expData_realq',...
        strcat(tmp_file, 'q', int2str(i), '.csv'));
    % Write wrench data
    writematrix(Data_cis(i).expData_w',...
        strcat(tmp_file,'w',int2str(i), '.csv'));
    % Write K
    writematrix(K, strcat(tmp_file,'k','.csv'));end
%}

%% For gen cis
