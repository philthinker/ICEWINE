clear variables;
% close all;
Common.clearClasses;

nTrials = 5;
basisNum = [3:4, 5:5:55];

for j=1:nTrials
    for i=1:length(basisNum)
        fprintf('Basis Trial %d Basis %d\n',j,basisNum(i));
        [cost_m_ProMPBasis(j,i), cost_std_ProMPBasis(j,i)] = TrajectoryGenerators.ProMPDemos.cFig_ctl_costEval(basisNum(i), 1);
        [cost_m_InvCtlBasis(j,i), cost_std_InvCtlBasis(j,i)] = TrajectoryGenerators.ProMPDemos.cFig_ctl_costEval(basisNum(i), 0);
        [cost_m_DMPBasis(j,i), cost_std_DMPBasis(j,i)] = TrajectoryGenerators.ProMPDemos.cFig_ctl_costEvalDMP(basisNum(i));
    end
end

demoNum = [2:4,5:5:25, 30:10:50, 50:25:100, 125:100:500];

for j = 1:nTrials
    for i = 1:length(demoNum)
        fprintf('Demo Trial %d Demos %d\n',j,demoNum(i));
        [cost_m_ProMPDemos(j,i), cost_std_ProMPDemos(j,i)] = TrajectoryGenerators.ProMPDemos.cFig_ctl_costEval(35, 1,demoNum(i));
        [cost_m_InvCtlDemos(j,i), cost_std_InvCtlDemos(j,i)] = TrajectoryGenerators.ProMPDemos.cFig_ctl_costEval(35, 0, demoNum(i));
        [cost_m_DMPDemos(j,i), cost_std_DMPDemos(j,i)] = TrajectoryGenerators.ProMPDemos.cFig_ctl_costEvalDMP(35,demoNum(i));
    end
end


%save('tempCostEval')

return


figure; hold on
errorbar(basisNum,mean(cost_m_ProMPBasis),mean(cost_std_ProMPBasis))
errorbar(basisNum,mean(cost_m_InvCtlBasis),mean(cost_std_InvCtlBasis))
errorbar(basisNum,mean(cost_m_DMPBasis),mean(cost_std_DMPBasis))
legend('ProMPs','Inv. Cov. Ctl','DMPs','Location','SE')
set(gca,'yscale','log')

xlabel('Number of basis functions', 'FontSize', 20);
ylabel('Cost', 'FontSize', 20);
set(gca, 'FontSize', 20);

fName = 'costCompBasis'
prefix='+TrajectoryGenerators/+ProMPDemos/+figs/';

Plotter.Matlab2Tikz.matlab2tikz('filename',[prefix,fName,'.tex'],'width','\figwidth',...
             'width','\figheight',...
             'extraAxisOptions',[...
             'ylabel style = {yshift = -1.5em,font = \normalsize},'...
             'xlabel style = {yshift =  0.3em,font = \normalsize},'...
             'xticklabel style = { font = \scriptsize},'...
             'yticklabel style = { font = \scriptsize},'
             ])
         
         
figure; hold on
errorbar(demoNum,mean(cost_m_ProMPDemos),mean(cost_std_ProMPDemos))
errorbar(demoNum,mean(cost_m_InvCtlDemos),mean(cost_std_InvCtlDemos))
errorbar(demoNum,mean(cost_m_DMPDemos),mean(cost_std_DMPDemos))
legend('ProMPs','Inv. Cov. Ctl','DMPs','Location','SE')
set(gca,'yscale','log')

xlabel('Number of demonstrations', 'FontSize', 20);
ylabel('Cost', 'FontSize', 20);
set(gca, 'FontSize', 20);

fName = 'costCompDemos'
prefix='+TrajectoryGenerators/+ProMPDemos/+figs/';

Plotter.Matlab2Tikz.matlab2tikz('filename',[prefix,fName,'.tex'],'width','\figwidth',...
             'width','\figheight',...
             'extraAxisOptions',[...
             'ylabel style = {yshift = -1.5em,font = \normalsize},'...
             'xlabel style = {yshift =  0.3em,font = \normalsize},'...
             'xticklabel style = { font = \scriptsize},'...
             'yticklabel style = { font = \scriptsize},'
             ])
