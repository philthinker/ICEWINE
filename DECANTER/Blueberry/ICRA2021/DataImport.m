%DataImport

%% OptitrackData init.

M = 6;
path = 'DECANTER\Blueberry\Data\10-26\00';
optiData = OptitrackData(2,8);
optiData = repmat(optiData,[1,M]);

for i = 1:M
    optiData(i) = optiData(i).readOptitrackData(path,i);
end
