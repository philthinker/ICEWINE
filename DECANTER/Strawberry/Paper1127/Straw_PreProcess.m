%Straw_PreProcess
%   The whole process of data pre-process
%   
%   Haopeng Hu
%   2020.11.27
%   All rights reserved
%   
%   Perseverance preveils!

% Data: 'Paper2011\Data\PCBA_dataPreprocess.mat'

%% Raw data show

% figure;
% for i = 1:M
%     tmpData = dataShow(i).body1raw(:,5:7);
%     plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3),'Color',Morandi_carnation(i));
%     hold on;
%     tmpData = dataShow(i).body2raw(:,5:7);
%     plot3(tmpData(:,1), tmpData(:,2), tmpData(:,3),'Color',Morandi_carnation(i));
% end
% grid on;
% axis equal; xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
% view(3);

% figure;
% for i = 1:7
%     subplot(7,1,i);
%     for j = 1:M
%         tmpData = dataShow(j).body1raw;
%         tmpT = dataShow(j).t;
%         plot(tmpT,tmpData(:,i),'Color',Morandi_carnation(j));
%         hold on;
%     end
%     ylabel(labels{i}); grid on;
%     if i == 7
%         xlabel('time(s)');
%     end
% end

%% Show one raw trajectory and its compensation

for i = 1:1
    figure;
    t = dataShow(i).t;
    tmpData = dataShow(i).body1rawGapped;
    for j = 1:7
        subplot(7,1,j);
        plot(t,tmpData(:,j),'Color',[0.0, 0.0, 1.0],'LineWidth',1.5);
        grid on;
        ylabel(labels{j});
    end
    xlabel('time(s)');
end

for i = 1:1
    figure;
    tmpData = dataShow(i).BodyDataGPComp1.DataTQuatPosi;
    t = tmpData(1,:);
    for j = 1:7
        subplot(7,1,j);
        plot(t,tmpData(j+1,:),'Color',[1.0, 0.0, 0.0],'LineWidth',1.5);
        hold on;
        plot(dataShow(i).t,dataShow(i).body1rawGapped(:,j),'Color',[0.0, 0.0, 1.0],'LineWidth',1.5);
        grid on;
        ylabel(labels{j});
    end
    xlabel('time(s)');
end
