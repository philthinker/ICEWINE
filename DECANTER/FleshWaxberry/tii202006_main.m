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
%   the initial joint position,
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
% % Generate K

N = size(Data_pre_w(1).expData_pPlus,2);
K = repmat(linspace(300,400,N)',[1,6]);
writematrix(K,'DECANTER\FleshWaxberry\Data\08-06\ret\pre\dataprek.csv');
%}

%% For gen pre

%% For demo-retrieve cis

%% For gen cis