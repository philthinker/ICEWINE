%BlackTea_live

%% For simplicity, extract the x-z data
%{
M = 9;
DataXZ = [];
DataXZ.SO2 = [];
DataXZ.p = [];
DataXZ = repmat(DataXZ,[1,M]);
DataXZApp = DataXZ;
DataXZAss = DataXZ;

for i = 1:M
    tmpPApp = permute(Demo(i).pre.se3([1,3],4,:), [1,3,2]);
    tmpPAss = permute(Demo(i).cis.se3([1,3],4,:), [1,3,2]);
    DataXZ(i).p = [tmpPApp, tmpPAss];
    DataXZApp(i).p = tmpPApp;
    DataXZAss(i).p = tmpPAss;
    tmpSO2App = Demo(i).pre.se3([1,3],[1,3],:);
    tmpSO2Ass = Demo(i).cis.se3([1,3],[1,3],:);
    DataXZ(i).SO2 = cat(3,tmpSO2App,tmpSO2Ass);
    DataXZApp(i).SO2 = tmpSO2App;
    DataXZAss(i).SO2 = tmpSO2Ass;
end

figure;
hold on;
for i = 1:M
    plot(DataXZ(i).p(1,:), DataXZ(i).p(2,:),'Color',Morandi_carnation(i));
end
axis equal; grid on;
xlabel('x(m)');  ylabel('z(m)');
%}

%% XZ data generalization
%{
DataXZAppGen = [];
DataXZAppGen.p0 = zeros(2,1);
DataXZAppGen.SO20 = eye(3);
DataXZAppGen = repmat(DataXZAppGen,[1,18]);
% For retrieval
for i = 1:9
    DataXZAppGen(i).p0 = DataXZApp(i).p(:,1);
    DataXZAppGen(i).SO20 = DataXZApp(i).SO2(:,:,1);
end
% For generalization
for i = 1:9
    DataXZAppGen(M+i).p0 = GeneralizedPoseApp(i).init_p([1,3],1);
    DataXZAppGen(M+i).SO20 = GeneralizedPoseApp(i).init_se3([1,3],[1,3],1);
end
%}
% More practical generalization
%{
MG = 18;
for i = 1:18
    DataXZAppGen(MG+i).p0 = [0.7-i*0.08, 0.015]';
    DataXZAppGen(MG+i).SO20 = DataXZ(1).SO2(:,:,end);
end
%}

%% Initial state determination
%{
DP = 2;
K = 1;
currP = [0,0]';
c = 1;
dState2 = zeros(1, K);
Mu = [1,1]';
Sigma = 2*repmat(eye(2),[1,1,1]);
for i = 1:K
    Xi = currP - Mu(1:DP,i);
    Sigmai = Sigma(1:DP, 1:DP, i);
    dState2(i) = c * (Xi' * (Sigmai \ Xi));
end
[~, StateID] = min(dState2);
%}

%% Simple GPR compensation test
%{
% function [dataOut, gapOut, IDOut] = simpleGPRCompen(dataIn,quatFlag, query, gprParam)
%simpleGPRCompen Simple data compensation by Gaussian process regression.
%   - Please set all the missing points as NaN.
%   - @GPZero is required.
%   dataIn: N x 3, N x 4 or N x 7 ..., MoCap data in the form of [x y z], [qw
%   qx qy qz] or [qw qx qy qz x y z].
%   quatFlag: Boolean, true for quat data 
%   query: N x 1, the query sequence (time) which is optional. If it is not
%   assigned, the indices will be the alternative.
%   gprParam: 1 x 3, the hyper-parameters of Gaussian process (default:[1e0,1e1,1e-2])
%   -------------------------------------------------
%   dataOut: N x 3, N x 4 or N x 7 ..., MoCap data in the form of dataIn.
%   gapOut: Nl x 3, Nl x 4 or Nl x 7 ..., the compensated MoCap data
%   IDOut: Nl x 1, the IDs of lost MoCap data

dataIn = Data(1).bodyMarker(1).marker{1};
quatFlag = false;
query = Data(1).time;
gprParam = [1e0,1e0,1e-2];

%% Initialization
N = size(dataIn,1);
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

%% Figure
figure;
for i = 1:D
    subplot(D,1,i);
    plot(query, dataIn(:,i), 'r');
    hold on;
    plot(query, dataOut(:,i), 'b');
    grid on;
end
%}

%% simpleGPRCompensation_bodyMarker test
%{
% function obj = simpleGPRCompensation_bodyMarker(obj, replaceFlag, gprParam)
%simpleGPCompensation_bodyMarker Replace the body and marker
%properties with the data in the bodyMarker property
%pre-processed by GPR.
%   replaceFlag: Boolean, true for replace all the data with
%   the estimated ones.
%   gprParam: 1 x 3, the hyper-param. of Gaussian process
%   (optional)
%   -----------------------------------------
%   obj: the object reference
obj = Data(1);
replaceFlag = false;
gprParam = [1e0,1e0,1e-3];
% if nargin < 3
%     gprParam = [1e0,1e1,1e-2];
% end
% Time as query
query = obj.time;
% Body data
for i = 1:obj.Nb
    [tmpData, ~, tmpID] = simpleGPRCompen(obj.bodyMarker(i).body, true, query, gprParam);
    if replaceFlag
        obj.body{i} = tmpData;
    else
        tmpBody = obj.bodyMarker(i).body;
        if ~isempty(tmpID)
            tmpBody(tmpID,:) = tmpData(tmpID,:);
        end
        obj.body{i} = tmpBody;
    end
end
% Marker data
% Recall that we always assume that the bodies share the same
% num. of markers on it.
tmpCnt = 1;
for i = 1:obj.Nb
    for j = 1:length(obj.bodyMarker(i).marker)
        [tmpData, ~, tmpID] = simpleGPRCompen(obj.bodyMarker(i).marker{j}, false, query, gprParam);
        if replaceFlag
            obj.marker{tmpCnt} = tmpData;
        else
            if isempty(tmpID)
                obj.marker{tmpCnt} = obj.bodyMarker(i).marker{j};
            else
                obj.marker{tmpCnt} = tmpData(tmpID,:);
            end
        end
        tmpCnt = tmpCnt + 1;
    end
end
%}

%% estimateQuatError test
%{
% function [Err,ErrList] = estimateQuatError(dataEsti,dataOrig)
%estimateQuatError Compute the estimation error of two unit quaternion
%trajectories of the same length.
%   dataEsti: N x 4, the estimated data [w x y z].
%   dataOrig: N x 4, the original data [w x y z].
%   -------------------------------------------------
%   Err: Scalar, the estimation error.
%   ErrList: N x 1, the estimation errors of each step.
dataEsti = [0 -1 0 0; 0 0 1 0; 0 0 0 1];
dataOrig = [1 0 0 0; 1 0 0 0; 1 0 0 0];
%% Data regulate
N = size(dataEsti,1);
dataEsti = quatNormalize(dataEsti(:,1:4)');        % 4 x N
dataOrig = quatNormalize(dataOrig(:,1:4)');     % 4 x N
tmpLogicIDs = dataEsti(1,:) < 0;
dataEsti(:,tmpLogicIDs) = -dataEsti(:,tmpLogicIDs);
tmpLogicIDs = dataOrig(1,:) < 0;
dataOrig(:,tmpLogicIDs) = -dataOrig(:,tmpLogicIDs);

%% Error computation
% Refer to "Morais, João Pedro, Svetlin Georgiev, and Wolfgang Sprößig.
% Real quaternionic calculus handbook. Springer Basel, 2014."
dataEstiConj = dataEsti;
dataEstiConj(2:4,:) = -dataEsti(2:4,:);
ErrList = zeros(N,1);
qErr = quatProduct(dataOrig,dataEstiConj); % 4 x N
for i = 1:N
    if all(qErr(:,i) == [-1, 0, 0, 0]')
        ErrList(i) = 2*pi;
    else
        ErrList(i) = 2*norm(quatlog(qErr(:,i)'));
    end
end
Err = mean(ErrList);
%}
