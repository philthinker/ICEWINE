%dataShow
%   Show the raw data
%
%   Haopeng Hu
%   2020.11.06
%   All rights reserved

%% Plot
%{
% Markers
ylabels = {'x','y','z'};
for i = 1:M
    figure;
    t = optiData(i).time;
    for j = 1:Nm
        tmpData = optiData(i).getMarkerData(j);
        for k = 1:3
            subplot(3,1,k);
            plot(t,tmpData(:,k));
            hold on;
            grid on;
            ylabel(ylabels{k});
        end
    end
end
%}
%{
% Body XYZ
ylabels = {'x','y','z'};
for i = 1:M
    figure;
    t = optiData(i).time;
    for j = 1:Nb
        tmpData = optiData(i).getBodyData(j);
        for k = 1:3
            subplot(3,1,k);
            plot(t,tmpData(:,k+4));
            hold on;
            grid on;
            ylabel(ylabels{k});
        end
    end
end

% Body Quat
ylabels = {'w','x','y','z'};
for i = 1:M
    figure;
    t = optiData(i).time;
    for j = 1:Nb
        tmpData = optiData(i).getBodyData(j);
        for k = 1:4
            subplot(4,1,k);
            plot(t,tmpData(:,k));
            hold on;
            grid on;
            ylabel(ylabels{k});
        end
    end
end
%}
%% Plot3
%{
% Marker

for i = 1:M
    figure;
    for j = 1:Nm
        tmpData = optiData(i).getMarkerData(j);
        plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3));
        hold on;
    end
    grid on; axis equal;
    view(3);
    xlabel('x'); ylabel('y'); zlabel('z');
end

% Body

for i = 1:M
    figure;
    for j = 1:Nb
        tmpData = optiData(i).getBodyData(j);
        plot3(tmpData(:,5), tmpData(:,6), tmpData(:,7));
        hold on;
    end
    grid on; axis equal;
    view(3);
    xlabel('x'); ylabel('y'); zlabel('z');
end
%}
