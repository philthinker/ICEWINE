%HoneyTea_main

%   Haopeng Hu
%   2019.12.29
%   All rights reserved

%% Test points layout planning

%{
% A area
dxA = 60;
dyA = 60;
NxA = floor(300/dxA) + 1;
NyA = floor(300/dyA);
omcA = OMCPerformance(0,[0,0,0],'LayoutA');
omcA = repmat(omcA,[NyA,NxA]);
tmpDx = 250;
tmpDy = 0+dyA;
for i = 1:NxA
    for j = 1:NyA
        omcA(j,i).position = [tmpDx,tmpDy,0];
        tmpDy = tmpDy + dyA;
    end
    tmpDx = tmpDx + dxA;
    tmpDy = 0+dyA;
end


% B1 area
dxB1 = 150;
dyB1 = 100;
NxB1 = floor(300/dxB1);
NyB1 = floor(300/dyB1);
omcB1 = OMCPerformance(0,[0,0,0],'LayoutB1');
omcB1 = repmat(omcB1,[NyB1,NxB1]);
tmpDx = 250+dxB1/2;
tmpDy = 600-dyB1/2;
for i = 1:NxB1
    for j = 1:NyB1
        omcB1(j,i).position = [tmpDx,tmpDy,0];
        tmpDy = tmpDy - dyB1;
    end
    tmpDx = tmpDx + dxB1;
    tmpDy = 600-dyB1/2;
end



% B2 area
dxB2 = 125;
dyB2 = 100;
NxB2 = floor(250/dxB2);
NyB2 = floor(600/dyB2);
omcB2 = OMCPerformance(0,[0,0,0],'LayoutB2');
omcB2 = repmat(omcB2,[NyB2,NxB2]);
tmpDx = dxB2/2;
tmpDy = dyB2/2;
for i = 1:NxB2
    for j = 1:NyB2
        omcB2(j,i).position = [tmpDx,tmpDy,0];
        tmpDy = tmpDy + dyB2;
    end
    tmpDx = tmpDx + dxB2;
    tmpDy = dyB2/2;
end

% B3 area
dxB3 = dxB2;
dyB3 = dyB2;
NxB3 = floor(250/dxB3);
NyB3 = floor(600/dyB3);
omcB3 = OMCPerformance(0,[0,0,0],'LayoutB3');
omcB3 = repmat(omcB3,[NyB3,NxB3]);
tmpDx = 800-dxB3/2;
tmpDy = dyB3/2;
for i = 1:NxB3
    for j = 1:NyB3
        omcB3(j,i).position = [tmpDx,tmpDy,0];
        tmpDy = tmpDy + dyB3;
    end
    tmpDx = tmpDx - dxB3;
    tmpDy = dyB3/2;
end
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
xyzC(2,:) = [800,0,0];
xyzC(3,:) = [800,600,0];
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

figure(2);
% A zone
N = size(omcA,1)*size(omcA,2)*size(omcA,3);
xyzA = zeros(N,3);
tmpid = 1;
for i = 1:size(omcA,1)
    for j = 1:size(omcA,2)
        for k = 1:size(omcA,3)
            xyzA(tmpid,:) = omcA(i,j,k).position;
            tmpid = tmpid + 1;
        end
    end
end
scatter3(xyzA(:,1),xyzA(:,2),xyzA(:,3),...
    'MarkerEdgeColor',[235/255,104/255,119/255],'MarkerFaceColor',[235/255,104/255,119/255]);
xlabel('x(mm)');    ylabel('y(mm)');    zlabel('z(mm)');
axis equal;
hold on;

% B zone
N = size(omcB1,1)*size(omcB1,2)*size(omcB1,3)+ ...
    size(omcB2,1)*size(omcB2,2)*size(omcB2,3)+...
    size(omcB3,1)*size(omcB3,2)*size(omcB3,3);
xyzB = zeros(N,3);
tmpid = 1;
for i = 1:size(omcB1,1)
    for j = 1:size(omcB1,2)
        for k = 1:size(omcB1,3)
            xyzB(tmpid,:) = omcB1(i,j,k).position;
            tmpid = tmpid + 1;
        end
    end
end
for i = 1:size(omcB2,1)
    for j = 1:size(omcB2,2)
        for k = 1:size(omcB2,3)
            xyzB(tmpid,:) = omcB2(i,j,k).position;
            tmpid = tmpid + 1;
        end
    end
end
for i = 1:size(omcB3,1)
    for j = 1:size(omcB3,2)
        for k = 1:size(omcB3,3)
            xyzB(tmpid,:) = omcB3(i,j,k).position;
            tmpid = tmpid + 1;
        end
    end
end
scatter3(xyzB(:,1),xyzB(:,2),xyzB(:,3),...
    'MarkerEdgeColor',[0,160/255,233/255],'MarkerFaceColor',[0,160/255,233/255]);
% Zone limit
xyzC = zeros(7,3);
xyzC(2,:) = [800,0,0];
xyzC(3,:) = [800,600,0];
xyzC(4,:) = [0,600,0];
xyzC(5:7,:) = xyzC(2:4,:) + [0,0,300];
scatter3(xyzC(:,1),xyzC(:,2),xyzC(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k');
%}

%% Position orders generation

% From x0 y0 z0
% We ordered the data and their objects by the ID property
% A zone
% B zone

%% Test

