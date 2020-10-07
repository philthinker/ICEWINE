%OMCS_main
%   Optical motion capture system
%   
%   Haopeng Hu
%   2020.10.02
%   All rights reserved

%% Read the Optitrack data

% load: data0930
optiData = pcba0929OptiData;

% Show
% % Show body data
% % % Plot3
figure;
for i = 1:M
    for j = 1:optiData(i).Nb
        tmpBody = optiData(i).getBodyData(j);
        tmpX = tmpBody(:,5);
        tmpY = tmpBody(:,6);
        tmpZ = tmpBody(:,7);
        plot3(tmpX,tmpY,tmpZ,'Color',Morandi_popsicle(i),'LineWidth',2.0);
        hold on;
    end
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

%% Interpolation to get rid of small gaps

% Show

%% Compensation to fullfill large gaps

% Show

%% alpha-TP-GMM

% Show

%% Control system simulation

% Show

