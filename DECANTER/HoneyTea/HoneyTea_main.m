%HoneyTea_main

%   Haopeng Hu
%   2019.12.29
%   All rights reserved

%% Test points layout planning
%{
% % A area
% dxA = 60;
% dyA = 60;
% NxA = floor(300/dxA);
% NyA = floor(300/dyA);
% omcA = OMCPerformance(0,[0,0,0],'LayoutA');
% omcA = repmat(omcA,[NyA,NxA]);
% tmpDx = 200+dxA/2;
% tmpDy = 0+dyA/2;
% for i = 1:NxA
%     for j = 1:NyA
%         omcA(j,i).position = [tmpDx,tmpDy,0];
%         tmpDy = tmpDy + dyA;
%     end
%     tmpDx = tmpDx + dxA;
%     tmpDy = 0+dyA/2;
% end
% 
% % B1 area
% dxB1 = 150;
% dyB1 = 150;
% NxB1 = floor(300/dxB1);
% NyB1 = floor(300/dyB1);
% omcB1 = OMCPerformance(0,[0,0,0],'LayoutB1');
% omcB1 = repmat(omcB1,[NyB1,NxB1]);
% tmpDx = 200+dxB1/2;
% tmpDy = 300+dyB1/2;
% for i = 1:NxB1
%     for j = 1:NyB1
%         omcB1(j,i).position = [tmpDx,tmpDy,0];
%         tmpDy = tmpDy + dyB1;
%     end
%     tmpDx = tmpDx + dxB1;
%     tmpDy = 300+dyB1/2;
% end
% 
% % B2 area
% dxB2 = 200;
% dyB2 = 150;
% NxB2 = floor(200/dxB2);
% NyB2 = floor(600/dyB2);
% omcB2 = OMCPerformance(0,[0,0,0],'LayoutB2');
% omcB2 = repmat(omcB2,[NyB2,NxB2]);
% tmpDx = dxB2/2;
% tmpDy = dyB2/2;
% for i = 1:NxB2
%     for j = 1:NyB2
%         omcB2(j,i).position = [tmpDx,tmpDy,0];
%         tmpDy = tmpDy + dyB2;
%     end
%     tmpDx = tmpDx + dxB2;
%     tmpDy = dyB2/2;
% end
% 
% 
% % B3 area
% dxB3 = dxB2;
% dyB3 = dyB2;
% NxB3 = floor(200/dxB3);
% NyB3 = floor(600/dyB3);
% omcB3 = OMCPerformance(0,[0,0,0],'LayoutB3');
% omcB3 = repmat(omcB3,[NyB3,NxB3]);
% tmpDx = 500 + dxB3/2;
% tmpDy = dyB3/2;
% for i = 1:NxB3
%     for j = 1:NyB3
%         omcB3(j,i).position = [tmpDx,tmpDy,0];
%         tmpDy = tmpDy + dyB3;
%     end
%     tmpDx = tmpDx + dxB3;
%     tmpDy = dyB3/2;
% end
%}
%{
figure(1);

% A zone
N = size(omcA,1)*size(omcA,2);
xyzA = zeros(N,3);
tmpid = 1;
for i = 1:size(omcA,1)
    for j = 1:size(omcA,2)
        xyzA(tmpid,:) = omcA(i,j).position;
        tmpid = tmpid + 1;
    end
end
scatter3(xyzA(:,1),xyzA(:,2),xyzA(:,3),'MarkerEdgeColor','r','MarkerFaceColor','r');
xlabel('x(mm)');    ylabel('y(mm)');    zlabel('z(mm)');
axis equal;
hold on;

% B zone
N = size(omcB1,1)*size(omcB1,2)+size(omcB2,1)*size(omcB2,2)+size(omcB3,1)*size(omcB3,2);
xyzB = zeros(N,3);
tmpid = 1;
for i = 1:size(omcB1,1)
    for j = 1:size(omcB1,2)
        xyzB(tmpid,:) = omcB1(i,j).position;
        tmpid = tmpid + 1;
    end
end
for i = 1:size(omcB2,1)
    for j = 1:size(omcB2,2)
        xyzB(tmpid,:) = omcB2(i,j).position;
        tmpid = tmpid + 1;
    end
end
for i = 1:size(omcB3,1)
    for j = 1:size(omcB3,2)
        xyzB(tmpid,:) = omcB3(i,j).position;
        tmpid = tmpid + 1;
    end
end
scatter3(xyzB(:,1),xyzB(:,2),xyzB(:,3),'MarkerEdgeColor','b','MarkerFaceColor','b');

% Zone limit
xyzC = zeros(7,3);
xyzC(2,:) = [700,0,0];
xyzC(3,:) = [700,600,0];
xyzC(4,:) = [0,600,0];
xyzC(5:7,:) = xyzC(2:4,:) + [0,0,300];
scatter3(xyzC(:,1),xyzC(:,2),xyzC(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k');
%}
%{
% omcA = repmat(omcA,[1,1,3]);
% omcB1 = repmat(omcB1,[1,1,3]);
% omcB2 = repmat(omcB2,[1,1,3]);
% omcB3 = repmat(omcB3,[1,1,3]);
% for i = 2:3
%     for j = 1:size(omcA,1)
%         for k = 1:size(omcA,2)
%             omcA(j,k,i).position = omcA(j,k,1).position + (i-1)*[0,0,150];
%         end
%     end
% end
% for i = 2:3
%     for j = 1:size(omcB1,1)
%         for k = 1:size(omcB1,2)
%             omcB1(j,k,i).position = omcB1(j,k,1).position + (i-1)*[0,0,150];
%         end
%     end
% end
% for i = 2:3
%     for j = 1:size(omcB2,1)
%         for k = 1:size(omcB2,2)
%             omcB2(j,k,i).position = omcB2(j,k,1).position + (i-1)*[0,0,150];
%         end
%     end
% end
% for i = 2:3
%     for j = 1:size(omcB3,1)
%         for k = 1:size(omcB3,2)
%             omcB3(j,k,i).position = omcB3(j,k,1).position + (i-1)*[0,0,150];
%         end
%     end
% end
%}
%{
% figure(2);
% % A zone
% N = size(omcA,1)*size(omcA,2)*size(omcA,3);
% xyzA = zeros(N,3);
% tmpid = 1;
% for i = 1:size(omcA,1)
%     for j = 1:size(omcA,2)
%         for k = 1:size(omcA,3)
%             xyzA(tmpid,:) = omcA(i,j,k).position;
%             tmpid = tmpid + 1;
%         end
%     end
% end
% scatter3(xyzA(:,1),xyzA(:,2),xyzA(:,3),...
%     'MarkerEdgeColor',[235/255,104/255,119/255],'MarkerFaceColor',[235/255,104/255,119/255]);
% xlabel('x(mm)');    ylabel('y(mm)');    zlabel('z(mm)');
% axis equal;
% hold on;
% 
% % B zone
% N = size(omcB1,1)*size(omcB1,2)*size(omcB1,3)+ ...
%     size(omcB2,1)*size(omcB2,2)*size(omcB2,3)+...
%     size(omcB3,1)*size(omcB3,2)*size(omcB3,3);
% xyzB = zeros(N,3);
% tmpid = 1;
% for i = 1:size(omcB1,1)
%     for j = 1:size(omcB1,2)
%         for k = 1:size(omcB1,3)
%             xyzB(tmpid,:) = omcB1(i,j,k).position;
%             tmpid = tmpid + 1;
%         end
%     end
% end
% for i = 1:size(omcB2,1)
%     for j = 1:size(omcB2,2)
%         for k = 1:size(omcB2,3)
%             xyzB(tmpid,:) = omcB2(i,j,k).position;
%             tmpid = tmpid + 1;
%         end
%     end
% end
% for i = 1:size(omcB3,1)
%     for j = 1:size(omcB3,2)
%         for k = 1:size(omcB3,3)
%             xyzB(tmpid,:) = omcB3(i,j,k).position;
%             tmpid = tmpid + 1;
%         end
%     end
% end
% scatter3(xyzB(:,1),xyzB(:,2),xyzB(:,3),...
%     'MarkerEdgeColor',[0,160/255,233/255],'MarkerFaceColor',[0,160/255,233/255]);
% % Zone limit
% xyzC = zeros(7,3);
% xyzC(2,:) = [700,0,0];
% xyzC(3,:) = [700,600,0];
% xyzC(4,:) = [0,600,0];
% xyzC(5:7,:) = xyzC(2:4,:) + [0,0,300];
% scatter3(xyzC(:,1),xyzC(:,2),xyzC(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k');
%}
%% Position orders generation

% From x0 y0 z0
% We ordered the data and their objects by the ID property
% A zone
% B zone

%% Linear trajectory generation

%{
% omcTrajA = OMCPerformance(0,[0 0 0],'TrajA');
% omcTrajA = repmat(omcTrajA,[2,5,3]);
% % The first row is Y-axis linear trajectory, while the second row is X-axis
% omcTrajA(1,:,:) = omcA(1,:,:);
% omcTrajA(2,:,:) = permute(omcA(:,1,:),[2,1,3]);
% for i = 1:2
%     for j = 1:5
%         for k = 1:3
%             omcTrajA(i,j,k).name = 'TrajA';
%             omcTrajA(i,j,k).position = omcTrajA(i,j,1).position + (k-1)*[0,0,150];
%         end
%     end
% end

% N = 100;
% xLineA = zeros(N,size(omcTrajA,1)*size(omcTrajA,2)*size(omcTrajA,3));
% yLineA = zeros(N,size(omcTrajA,1)*size(omcTrajA,2)*size(omcTrajA,3));
% zLineA = zeros(N,size(omcTrajA,1)*size(omcTrajA,2)*size(omcTrajA,3));
% LA = (NxA-1) * dxA;
% tmpid = 1;
% for k = 1:3
%     for i = 1:2
%         for j = 1:size(omcTrajA,2)
%             tmpPosi = omcTrajA(i,j,k).position;
%             if i == 1
%                 % Y-axis linear traj.
%                 xLineA(:,tmpid) = ones(N,1)*tmpPosi(1);
%                 yLineA(:,tmpid) = linspace(tmpPosi(2),tmpPosi(2)+LA,N);
%                 zLineA(:,tmpid) = ones(N,1)*tmpPosi(3);
%                 tmpid = tmpid + 1;
%             elseif i == 2
%                 % X-axis linear traj.
%                 xLineA(:,tmpid) = linspace(tmpPosi(1),tmpPosi(1)+LA,N);
%                 yLineA(:,tmpid) = ones(N,1)*tmpPosi(2);
%                 zLineA(:,tmpid) = ones(N,1)*tmpPosi(3);
%                 tmpid = tmpid + 1;
%             end
%         end
%     end
% end
%}
%{
% omcTrajB = OMCPerformance(0,[0,0,0],'TrajB');
% omcTrajB = repmat(omcTrajB,[2,2]);
% % The first row is Y-axis linear trajectory, while the second row is X-axis
% omcTrajB(1,1).position = omcB2(1,1,1).position;
% omcTrajB(1,2).position = omcB3(1,1,1).position;
% omcTrajB(2,1).position = omcB2(3,1,1).position;
% omcTrajB(2,2).position = omcB2(4,1,1).position;
% omcTrajB = repmat(omcTrajB,[1,1,3]);
% for i = 1:2
%     for j = 1:2
%         for k = 2:3
%             omcTrajB(i,j,k).position = omcTrajB(i,j,1).position + (k-1)*[0,0,150];
%         end
%     end
% end

% xLineB = zeros(N,size(omcTrajB,1)*size(omcTrajB,2)*size(omcTrajB,3));
% yLineB = zeros(N,size(omcTrajB,1)*size(omcTrajB,2)*size(omcTrajB,3));
% zLineB = zeros(N,size(omcTrajB,1)*size(omcTrajB,2)*size(omcTrajB,3));
% LBy = 450;
% LBx = 500;
% tmpid = 1;
% for k = 1:3
%     xLineB(:,tmpid) = omcTrajB(1,1,k).position(1) * ones(N,1);
%     xLineB(:,tmpid+1) = omcTrajB(1,2,k).position(1) * ones(N,1);
%     xLineB(:,tmpid+2) = linspace(omcTrajB(2,1,k).position(1),omcTrajB(2,1,k).position(1)+LBx,N);
%     xLineB(:,tmpid+3) = linspace(omcTrajB(2,2,k).position(1),omcTrajB(2,2,k).position(1)+LBx,N);
%     yLineB(:,tmpid) = linspace(omcTrajB(1,1,k).position(2),omcTrajB(1,1,k).position(2)+LBy,N);
%     yLineB(:,tmpid+1) = linspace(omcTrajB(1,2,k).position(2),omcTrajB(1,2,k).position(2)+LBy,N);
%     yLineB(:,tmpid+2) = omcTrajB(2,1,k).position(2) * ones(N,1);
%     yLineB(:,tmpid+3) = omcTrajB(2,2,k).position(2) * ones(N,1);
%     zLineB(:,tmpid) = omcTrajB(1,1,k).position(3) * ones(N,1);
%     zLineB(:,tmpid+1) = omcTrajB(1,2,k).position(3) * ones(N,1);
%     zLineB(:,tmpid+2) = omcTrajB(2,1,k).position(3) * ones(N,1);
%     zLineB(:,tmpid+3) = omcTrajB(2,2,k).position(3) * ones(N,1);
%     tmpid = tmpid + 4;
% end
%}
%{
figure(3);
% A zone
for i = 1:size(xLineA,2)
    plot3(xLineA(:,i),yLineA(:,i),zLineA(:,i),'Color',[235/255,104/255,119/255]);
    hold on;
end
axis equal; grid on;
xlabel('x(mm)');    ylabel('y(mm)');    zlabel('z(mm)');
% B zone
for i = 1:size(xLineB,2)
    plot3(xLineB(:,i),yLineB(:,i),zLineB(:,i),'Color',[0,160/255,233/255]);
end
scatter3(xyzC(:,1),xyzC(:,2),xyzC(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k');
%}

%% Circle trajectory generation

% omcCircA = OMCPerformance(0,[350,150,0],'CircA');
% omcCircA = repmat(omcCircA,[1,4]);
% % The 1st, 2nd and 3rd are horizental circles, while the 4th is the
% % vertical half cirlce.
% omcCircA(1,2).position = omcCircA(1,1).position+[0,0,150];
% omcCircA(1,3).position = omcCircA(1,1).position+[0,0,300];

R = 150;
N = 500; tmpTheta = linspace(0,2*pi,N)';
xCirc = zeros(N,size(omcCircA,2));
yCirc = zeros(N,size(omcCircA,2));
zCirc = zeros(N,size(omcCircA,2));

xCirc(:,1:3) = repmat(350 + R*cos(tmpTheta),[1,3]);
yCirc(:,1:3) = repmat(150 + R*sin(tmpTheta),[1,3]);
for k = 2:3
    zCirc(:,k) = omcCircA(k).position(3)*ones(N,1);
end

tmpTheta = linspace(0,pi,N)';
xCirc(:,4) = 350 + R*cos(tmpTheta);
yCirc(:,4) = 150 * ones(N,1);
zCirc(:,4) = 0 + R*sin(tmpTheta);

figure(4);

for i = 1:size(xCirc,2)-1
    plot3(xCirc(:,i),yCirc(:,i),zCirc(:,i),'Color',[235/255,104/255,119/255]);
    hold on;
end
plot3(xCirc(:,4),yCirc(:,4),zCirc(:,4),'Color',[0,160/255,233/255]);
axis equal; grid on;
xlabel('x(mm)');    ylabel('y(mm)');    zlabel('z(mm)');
scatter3(xyzC(:,1),xyzC(:,2),xyzC(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k');