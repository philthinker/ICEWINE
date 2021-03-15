function [dataOut, gapOut, IDOut] = simpleGPRCompen(dataIn,quatFlag, query, gprParam)
%simpleGPRCompen Simple data compensation by Gaussian process regression.
%   - Please set all the missing points as NaN.
%   - @GPZero is required.
%   dataIn: N x 3, N x 4 or N x 7 ..., MoCap data in the form of [x y z], [qw
%   qx qy qz] or [qw qx qy qz x y z].
%   quatFlag: Boolean, true for quat data 
%   query: N x 1, the query sequence (time) which is optional. If it is not
%   assigned, the indices will be the alternative.
%   gprParam: 1 x 3, the hyper-parameters of Gaussian process (default:[1e0,1e0,1e-2])
%   -------------------------------------------------
%   dataOut: N x 3, N x 4 or N x 7 ..., MoCap data in the form of dataIn.
%   gapOut: Nl x 3, Nl x 4 or Nl x 7 ..., the compensated MoCap data
%   IDOut: Nl x 1, the IDs of lost MoCap data

%% Initialization
N = size(dataIn,1);
if nargin < 3
    % No query assigned
    query = (1:N)';
end
if nargin < 4
    % Default GP hyper-param.
    gprParam = [1e0,1e0,1e-2];
end
D = size(dataIn, 2);
dataOut = [];
gapOut = [];

%% Find the lost IDs
tmpID = (1:N);
LIDOut = isnan(dataIn(:,1));
if any(LIDOut)
    % There are lost points
    IDOut = tmpID(LIDOut);
else
    % No lost point
    IDOut = [];
end
dataRest = dataIn(~isnan(dataIn(:,1)),:)';      % D x N-Nl
queryRest = query(~isnan(dataIn(:,1)))';       % 1 x N-Nl

%% Regualte the data
if quatFlag
    % Find the auxiliary quaternion.
    tmpID = N;
    while tmpID > 1
        if isnan(dataIn(tmpID,1))
            tmpID = tmpID - 1;
        else
            break;
        end
    end
    if D == 4
        % [qw qx qy qz]
        qa = dataIn(tmpID,:)';
        dataRestEta = quatLogMap(dataRest, qa);
        dataRest = dataRestEta;     % [etax; etay; etaz]
    elseif D == 7
        % [qw qx qy qz x y z]
        qa = dataIn(tmpID,1:4)';
        tmpData = dataRest(1:4,:);
        dataRestEta = quatLogMap(tmpData,qa);
        tmpData = dataRest(5:7,:);
        dataRest = [dataRestEta; tmpData];  % [etax; etay; etaz; x; y; z]
    else
        return;
    end
end

%% GP initialization

model = GPZero([queryRest; dataRest]);
model = model.setParam(gprParam(1), gprParam(2), gprParam(3));
model = model.preGPR();

%% GPR compensation

expOut = model.GPR(query');
tmpData = expOut.Data;
tmpData = tmpData(2:end,:);

%% Dataout regulate

dataOut = tmpData;
if quatFlag
    if D == 4
        dataOutQuat = quatExpMap(tmpData,qa);
        dataOut = dataOutQuat;
    elseif D == 7
        dataOutQuat = quatExpMap(tmpData(1:3,:),qa);
        dataOut = [dataOutQuat; tmpData(4:6,:)];
    end
end
dataOut = dataOut';
if isempty(IDOut)
    gapOut = [];
else
    gapOut = dataOut(IDOut,:);
end

end

