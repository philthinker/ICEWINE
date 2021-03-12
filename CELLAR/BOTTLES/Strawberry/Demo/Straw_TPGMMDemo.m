%Honeytea_main

% load: data0930, or data0930_honeytea
% optiData = pcba0929OptiData;
% for i = 1:M
%     optiData(i) = optiData(i).quatWXYZ();
% end

%% Show raw data
%{
% % Show body data
% % % Plot3
figure;
for i = 1:M
    for j = 1:optiData(i).Nb
        tmpBody = optiData(i).getBodyData(j);
        tmpX = tmpBody(:,5);
        tmpY = tmpBody(:,6);
        tmpZ = tmpBody(:,7);
        plot3(tmpX,tmpY,tmpZ,'Color',Morandi_popsicle(i),'LineWidth',1.5);
        hold on;
    end
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);
%}

%% TP-Data prepare

% Data prepare
%{
% % Data struct
Demos = [];
Demos.rawData_l = [];
Demos.rawData_r = [];
Demos = repmat(Demos,[1,M]);
% % Initialization
for i = 1:M
    % Note that we always assume the *left hand* data is body 1
    % while the *right hand* data is the body 2.
    Demos(i).rawData_l = optiData(i).getGapFreeBodyData(1); Demos(i).rawData_l = Demos(i).rawData_l(150:end,:);
    Demos(i).rawData_r = optiData(i).getGapFreeBodyData(2); Demos(i).rawData_r = Demos(i).rawData_r(150:end,:);
end

figure;
for i = 1:M
    tmpData = Demos(i).rawData_l(:,5:7);
    plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3), 'Color', Morandi_hydrangea(i));
    hold on;
    tmpData = Demos(i).rawData_r(:,5:7);
    plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3), 'Color', Morandi_hydrangea(i));
end
axis equal; grid on;
view(3);
%}

% TP-Data prepare
%{
for i = 1:M
    % We always assume that the part in the left hand is the one to be fixed
    % during assembly process.
    % The moving part w.r.t. the fixed part.
    tmpData_l = Demos(i).rawData_l;
    tmpData_r = Demos(i).rawData_r;
    tmpN = size(tmpData_l,1);
    tmpData = zeros(4,4,tmpN);
    for j = 1:tmpN
        Tol = SO3P2SE3(quat2rotm(tmpData_l(j,1:4)), tmpData_l(j,5:7)');
        Tor = SO3P2SE3(quat2rotm(tmpData_r(j,1:4)), tmpData_r(j,5:7)');
        Tlr = Tol\Tor;
        tmpData(:,:,j) = Tlr;
    end
    Demos(i).fixedPartBaseData = tmpData;
    % The transformed moving part w.r.t. its inital and goal pose.
    Demos(i).movingPartInitData = tmpData;
    Demos(i).movingPartGoalData = tmpData;
    for j = 1:tmpN
        Demos(i).movingPartInitData(:,:,j) = tmpData(:,:,1)\tmpData(:,:,j);
        Demos(i).movingPartGoalData(:,:,j) = tmpData(:,:,end)\tmpData(:,:,j);
    end
    Demos(i).A = repmat(eye(4),[1,1,2]);    % A(:,:,1) is the init. orientation, A(:,:,2) goal orientation
    Demos(i).b = zeros(4,2);                      % b(:,1) is the init. position, b(:,2) goal position
    Demos(i).A(2:4,2:4,1) = tform2rotm(tmpData(:,:,1)');
    Demos(i).A(2:4,2:4,2) = tform2rotm(tmpData(:,:,end)');
    Demos(i).b(2:4,1) = tmpData(1:3,4,1);
    Demos(i).b(2:4,2) = tmpData(1:3,4,end);
    Demos(i).TPData_init = permute(Demos(i).movingPartInitData(1:3,4,:),[1,3,2]);
    Demos(i).TPData_goal = permute(Demos(i).movingPartGoalData(1:3,4,:),[1,3,2]);
end
%}

% % Time variable
%{
queryMin = 0;
queryMax = 0.5;
for i = 1:M
    tmpN = size(Demos(i).TPData_goal,2);
    tmpQuery = linspace(queryMin, queryMax,tmpN);
    Demos(i).data = [tmpQuery; permute(Demos(i).fixedPartBaseData(1:3,4,:), [1,3,2]) ];
    Demos(i).query = tmpQuery;
    Demos(i).TPData = zeros(4,2,tmpN);
    for j = 1:tmpN
        Demos(i).TPData(:,1,j) = [tmpQuery(j); Demos(i).TPData_init(:,j)];
        Demos(i).TPData(:,2,j) = [tmpQuery(j); Demos(i).TPData_goal(:,j)];
    end
end
%}
% % Phase variable
%{
queryMin = 0;
queryMax = 0.5;
for i = 1:M
    tmpData = permute(Demos(i).fixedPartBaseData(1:3,4,:), [1,3,2]);
    tmpN = size(tmpData,2);
    % Phase computation
    tmpLength = norm(tmpData(:,1) - tmpData(:,end));
    tmpQuery = zeros(1,tmpN);
    for j = 1:tmpN
        tmpQuery(j) = (norm(tmpData(:,j) - tmpData(:,end))/tmpLength)*(queryMax - queryMin) + queryMin;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
    Demos(i).data = [tmpQuery; tmpData ];
    Demos(i).query = tmpQuery;
    Demos(i).TPData = zeros(4,2,tmpN);
    for j = 1:tmpN
        Demos(i).TPData(:,1,j) = [tmpQuery(j); Demos(i).TPData_init(:,j)];
        Demos(i).TPData(:,2,j) = [tmpQuery(j); Demos(i).TPData_goal(:,j)];
    end
end
%}

% Show TP demos
%{
figure;
for i = 1:M
    plot3(Demos(i).TPData_init(1,:), Demos(i).TPData_init(2,:), Demos(i).TPData_init(3,:),...
        'Color',Morandi_popsicle(1),'LineWidth',1.5);
    hold on;
end
grid on; axis equal; view(131,53);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');

figure;
for i = 1:M
    plot3(Demos(i).TPData_goal(1,:), Demos(i).TPData_goal(2,:), Demos(i).TPData_goal(3,:),...
        'Color',Morandi_popsicle(2),'LineWidth',1.5);
    hold on;
end
grid on; axis equal; view(131,53);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
%}

%% TP-GMM and alpha-TP-GMR

% TP-GMM and EM algorithm

% policy = FWTPGMMZero(6,4,2);
% policy = policy.initGMMTimeBased(Demos);
% policy = policy.learnGMM(Demos);

% % Show TP-GMM (Be patient)
%{
movingPartInit.Mu = permute(policy.Mus(2:4,1,:), [1,3,2]);
movingPartInit.Sigma = permute(policy.Sigmas(2:4,2:4,1,:),[1,2,4,3]);

figure;
plotGMM3SC(movingPartInit.Mu,movingPartInit.Sigma,Morandi_popsicle(1),0.4,1);
hold on;
for i = 1:M
    tmpData = Demos(i).TPData_init;
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color',[0.5,0.5,0.5],'LineWidth',1.5);
end
grid on; axis equal;
view(3);

movingPartGoal.Mu = permute(policy.Mus(2:4,2,:), [1,3,2]);
movingPartGoal.Sigma = permute(policy.Sigmas(2:4,2:4,2,:),[1,2,4,3]);

figure;
plotGMM3SC(movingPartGoal.Mu,movingPartGoal.Sigma,Morandi_popsicle(1),0.4,1);
hold on;
for i = 1:M
    tmpData = Demos(i).TPData_goal;
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color',[0.5,0.5,0.5],'LineWidth',1.5);
end
grid on; axis equal;
view(3);
%}

% TP-GMR
%{
motionData.query = linspace(queryMax,queryMin,200);
[motionData.expData, motionData.expSigma, motionData.alpha] = policy.GMR(motionData.query, motionData.queryFrame);
%}

% % Show robot motion

figure;
tmpData = motionData.expData;
plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color',Morandi_popsicle(3),'LineWidth',2.0);
hold on;
legend('Robot motion trajectory');
for i = 1:M
    tmpData = Demos(i).TPData_init;
    tmpData = motionData.queryFrame(1).A(2:4,2:4) * tmpData + motionData.expData(:,1);
    plot3(tmpData(1,:),tmpData(2,:),tmpData(3,:),'Color',Morandi_popsicle(1));
    tmpData = Demos(i).TPData_goal;
    tmpData = motionData.queryFrame(2).A(2:4,2:4) * tmpData + motionData.expData(:,end);
    plot3(tmpData(1,:),tmpData(2,:),tmpData(3,:),'Color',Morandi_popsicle(2));
end
grid on; axis equal;
axis([-Inf, Inf,-Inf, 0.15, 0, 0.5]);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

% figure;
% plot(motionData.query, motionData.alpha);
% grid on; grid on;
%}