%MCA_dualarmMotionGen

% Data: 'Data\MCA_DualArmTraj'

%% Show raw data
%{
figure;
for i = 1:M
    tmpData = frankaData(i).p;
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'Color', Morandi_popsicle(i));
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

figure;
ylabels = {'v', 'u_{x}', 'u_{y}', 'u_{z}'};
for i = 1:M
    tmpq = frankaData(i).q;
    tmpt = frankaData(i).time;
    for j = 1:4
        subplot(4,1,j);
        plot(tmpt,tmpq(j,:),'Color',Morandi_popsicle(i));
        hold on; grid on;
        ylabel(ylabels{j});
    end
end
%}

%% GMR for Panda

% GMR
%{
query = linspace(0,1,1000);
[exp_pPanda, Sigma_pPanda] = MTD.pPolicy.GMR(query);
%}

% Show
%{
figure;
for i = 1:M
    tmpData = frankaData(i).p;
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', Morandi_popsicle(1));
    hold on;
end
plot3(exp_pPanda(1,:), exp_pPanda(2,:), exp_pPanda(3,:),'Color', Morandi_popsicle(2),'LineWidth',3.0);
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
criticalID = [1,100,200,300,305,400,1000];
% scatter3(exp_pPanda(1,criticalID), exp_pPanda(2,criticalID), exp_pPanda(3,criticalID),...
%     100,[1.0,0.0,0.0],'filled');
%}

%% QGMR for Panda

% GMR
%{
[exp_etaPanda, Sigma_etaPanda] = MTD.qPolicy.GMR(query);
exp_qPanda = MTD.qPolicy.expmap(exp_etaPanda);
%}

% Show
%{
ylabels = {'q_w','q_x','q_y','q_z'};
figure;
for j = 1:4
    subplot(4,1,j);
    for i = 1:M
        tmpData = frankaData(i).q;
        t = frankaData(i).time;
        plot(t, tmpData(j,:),'Color',Morandi_popsicle(1));
        hold on;
    end
    plot(query, exp_qPanda(j,:),'Color',Morandi_popsicle(2),'LineWidth',3.0);
    ylabel(ylabels{j}); grid on;
%     scatter(query(criticalID), exp_qPanda(j,criticalID),...
%     100,[1.0,0.0,0.0],'filled');
end
xlabel('Phase');
%}


