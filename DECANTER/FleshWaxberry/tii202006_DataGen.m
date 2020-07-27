%tii202006_DataGen
%   Generate the data used in real experiments
%   

%% Policy Pre Retrieve P

% load('Data/lead0722')
csvwrite('ret_p_01.csv',toPandaCarte(Data_pre_w(1).expData_p,testOTEE,1));

%% Policy Pre Retrieve P,Q