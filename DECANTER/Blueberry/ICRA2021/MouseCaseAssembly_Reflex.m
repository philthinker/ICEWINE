% MouseCaseAssembly_Reflex

% dataReflex = [];
% % x y z alpha_y beta_x beta_y beta_z
% % mm, deg
% dataReflex.reflex = [
%     [40,-2.5,-540,-5,-13.0257,8.2755,18.5927];
%     [40,-2.5,-556,-5,-13.0257,8.2755,18.5927];
%     [37,-3,-556,-4,-12.0687,7.9556,18.7278];
%     [33,-3,-557,-3,-11.1132,7.6334,18.8574];
%     [30,-4,-557,-2,-10.1591,7.3092,18.9814];
%     [25,-5,-557,-1,-9.2065,6.9829,19.1000];
%     [10,-6,-561,0,-8.2551,6.6548,19.2129];
%     [8,-6,-564,0,-8.2551,6.6548,19.2129];
%     [5,-7,-564,0,-8.2551,6.6548,19.2129];
%     [3,-7,-566,1,-7.3050,6.3248,19.3203];
%     [1,-7,-566.5,2,-6.3562,5.9931,19.4221];
%     [0.5,-7,-566.5,3,-5.6024,5.4679,17.5181];
%     [1.5,-7,-566.5,6,-5.6024,5.4679,17.5181]
%     ];
% N = size(dataReflex.reflex,1);
% dataReflex.pM = dataReflex.reflex(:,1:3)'/1000;
% dataReflex.eulM = zeros(3,N);
% dataReflex.eulM(2,:) = deg2rad(dataReflex.reflex(:,4)');
% dataReflex.eulS = deg2rad(dataReflex.reflex(:,5:7)');

% dataReflex.SO3M = eul2rotm(dataReflex.eulM');
% dataReflex.SO3S = eul2rotm(dataReflex.eulS');

% dataReflex.SE3M = SO3P2SE3(dataReflex.SO3M,dataReflex.pM);
% dataReflex.SE3S = rotm2tform(dataReflex.SO3S);

% dataReflex.qM = rotm2quat(dataReflex.SO3M);
% dataReflex.qS = rotm2quat(dataReflex.SO3S);

%% ctraj

% dataReflex.N = 10000;
% dataReflex.dist = zeros(1, size(dataReflex.reflex,1)-1);
% for i = 2:size(dataReflex.reflex,1)
%     dataReflex.dist(i-1) = norm(dataReflex.pM(:,i)*1000 - dataReflex.pM(:,i-1)*1000);
% end

% tmpN = zeros(1,size(dataReflex.dist,2));
% tmpDataM = cell(1,size(dataReflex.dist,2));
% tmpDataS = tmpDataM;
% for i = 1:size(dataReflex.dist,2)
%     tmpN(i) = round(dataReflex.N * dataReflex.dist(i) / sum(dataReflex.dist));
%     tmpDataM{i} = ctraj(dataReflex.SE3M(:,:,i), dataReflex.SE3M(:,:,i+1), tmpN(i));
%     tmpDataS{i} = ctraj(dataReflex.SE3S(:,:,i), dataReflex.SE3S(:,:,i+1), tmpN(i));
% end
% paraData.reflexSE3M = zeros(4,4,sum(tmpN));
% paraData.reflexSE3S = zeros(4,4,sum(tmpN));
% tmpCounter = 1;
% for i = 1:size(dataReflex.dist,2)
%     paraData.reflexSE3M(:,:,tmpCounter:tmpCounter+tmpN(i)-1) = tmpDataM{i};
%     paraData.reflexSE3S(:,:,tmpCounter:tmpCounter+tmpN(i)-1) = tmpDataS{i};
%     tmpCounter = tmpCounter + tmpN(i);
% end

% figure;
% tmpIndex = 255;
% tmpData = permute(paraData.reflexSE3M(1:3,4,:), [1,3,2])*1000;
% plot3(tmpData(1,tmpIndex:end), tmpData(2,tmpIndex:end),tmpData(3,tmpIndex:end));
% grid on; axis equal;
% xlabel('x(mm)'); ylabel('y(mm)'); zlabel('z(mm)');
% view(3);
% 
% figure;
% t = linspace(0,1,size(tmpData,2));
% for i = 1:3
%     subplot(3,1,i);
%     plot(t, tmpData(i,:));
% end

% figure;
% tmpData = tform2quat( paraData.reflexSE3M(:,:,tmpIndex:end));
% t = linspace(1,0,size(tmpData,1));
% ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
% for i = 1:4
%     subplot(4,1,i);
%     plot(t, tmpData(:,i));
%     ylabel(ylabels{i});
%     grid on;
% end

figure;
tmpData = tform2quat( paraData.reflexSE3S(:,:,tmpIndex:end));
t = linspace(1,0,size(tmpData,1));
ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
for i = 1:4
    subplot(4,1,i);
    plot(t, tmpData(:,i));
    ylabel(ylabels{i});
    grid on;
end

% paraData.reflex = zeros(size(paraData.reflexSE3M,3), 7);
% paraData.reflex(:,1:3) = permute( paraData.reflexSE3M(1:3,4,:), [1,3,2])'*1000;
% tmpData = rad2deg( tform2eul(paraData.reflexSE3M) );
% paraData.reflex(:,4) = tmpData(:,2);
% paraData.reflex(:,5:7) = rad2deg( tform2eul(paraData.reflexSE3S) );
% 
% tmpData = paraData.reflex;

