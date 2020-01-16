%HoneyTea_main0116

%% Data read

% Data.time = [];
% Data.fps = [];
% Data.position = [];
% Data = repmat(Data,[1,6]);
% for i = 1:6
%     Data(i) = readNOKOV(eval(strcat('test',int2str(i))),60);
% end

% DataLineP1.time = [];
% DataLineP1.fps = [];
% DataLineP1.position = [];
% DataLineP1 = repmat(DataLineP1,[1,5]);

%% Static test

omc = OMCPerformance(1,0,'Simple0115');
M = 6;

for i = 1:M
    Data(i).mean = mean(Data(i).position,1);
    Data(i).SD = omc.SDCompute(Data(i).position);
end

for i = 3:M-1
    Data(i).MAE = abs(norm(Data(i).mean - Data(i+1).mean) - 5);
end

for i = 4:M
    Data(i).RMSE = omc.RMSEComptue(Data(i).position,repmat(Data(3).mean+[0,0,5]*(i-3),[size(Data(i).position,1),1]));
end
