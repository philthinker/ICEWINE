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

% Show TP demos
%{
figure;
for i = 1:M
    plot3(Demos(i).TPData_init(1,:), Demos(i).TPData_init(2,:), Demos(i).TPData_init(3,:),...
        'Color',Morandi_popsicle(1),'LineWidth',1.5);
    hold on;
end
grid on; axis equal; view(3);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');

figure;
for i = 1:M
    plot3(Demos(i).TPData_goal(1,:), Demos(i).TPData_goal(2,:), Demos(i).TPData_goal(3,:),...
        'Color',Morandi_popsicle(2),'LineWidth',1.5);
    hold on;
end
grid on; axis equal; view(3);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
%}

%% TP-GMM and alpha-TP-GMR

% TP-GMM and EM algorithm

policy = FWTPGMMZero(5,4,2);

% Show TP-GMM

 

% Show robot motion


