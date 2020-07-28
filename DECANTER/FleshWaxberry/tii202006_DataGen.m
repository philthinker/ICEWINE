%tii202006_DataGen
%   Generate the data used in real experiments
%   

%% Policy Pre Retrieve P

% load('Data/lead0722')

% csvwrite('ret01pre_p.csv',toPandaCarte(Data_pre_w(1).expData_p));
% csvwrite('ret01pre_q.csv',toPandaCarte(Data_pre_q(1).expData_realq));
% csvwrite('ret01pre_p_plus.csv',toPandaCarte(Data_pre_w(1).expData_pPlus));
% csvwrite('ret01pre_q_plus.csv',toPandaCarte(Data_pre_q(1).expData_realqPlus));

% % Comparison
figure;
for i = 1:M
    tmpX = Demos_pre(i).data(2,:);
    tmpY = Demos_pre(i).data(3,:);
    tmpZ = Demos_pre(i).data(4,:);
    plot3(tmpX,tmpY,tmpZ,'Color',[0.6, 0.6, 0.6]);
    hold on;
end
grid on; axis equal;

tmpX = Data_pre_w(1).expData_pPlus(1,:);
tmpY = Data_pre_w(1).expData_pPlus(2,:);
tmpZ = Data_pre_w(1).expData_pPlus(3,:);
plot3(tmpX,tmpY,tmpZ,'Color',[0.0, 0.0, 0.6]);

tmpX = testpre01(:,20);
tmpY = testpre01(:,21);
tmpZ = testpre01(:,22);
plot3(tmpX,tmpY,tmpZ,'Color',[0.6, 0.0, 0.0]);

figure;
t = linspace(0,1,size(testpre01,1));
for j = 1:7
    subplot(7,1,j);
    plot(t,testpre01(:,j));
end

figure;
for i = 1:4
    subplot(4,1,i);
    for j = 1:M
        tmpN = size(Demos_pre(j).q,2);
        t = (1:tmpN)*dt - dt;
        tmpq = quatRegulate(Demos_pre(j).q);
        plot(t, tmpq(i,:),'Color',[0.6, 0.6, 0.6]);
        hold on;
    end
    grid on;
end

%% Policy Pre Retrieve P,Q