%tii202006_DataGen
%   Generate the data used in real experiments
%   

%% Policy Pre Retrieve P

% load('Data/lead0722')

csvwrite('ret01pre_p.csv',toPandaCarte(Data_pre_w(1).expData_p));
csvwrite('ret01pre_q.csv',toPandaCarte(Data_pre_q(1).expData_realq));

%% Policy Pre Retrieve P,Q