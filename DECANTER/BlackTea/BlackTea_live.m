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