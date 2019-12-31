function prepare_plot(fName,savePlot)

title('')
set(gca, 'XTickLabel', num2cell(0:0.25:1))
% set(gcf, 'Position', [580 549 643 329]);
xlabel('time(s)', 'FontSize', 20);
ylabel('q(rad)', 'FontSize', 20);
set(gca, 'FontSize', 20);
% axis([0 200 -1.2 1.5])

prefix='+TrajectoryGenerators/+ProMPDemos/+figs/';

if (savePlot)
%     Plotter.plot2svg([fName,'.svg'], gcf) %Save figure
%     set(gcf,'Renderer','painters')
%     set(gcf, 'PaperPosition', [0 0 8 5]); %Position plot at left hand corner with width 5 and height 5.
%     set(gcf, 'PaperSize', [8 5]); %Set the paper to have width 5 and height 5.
%     saveas(gcf, fName, 'pdf') %Save figure
Plotter.Matlab2Tikz.matlab2tikz('filename',[prefix,fName,'.tex'],'width','\figwidth',...
             'width','\figheight',...
             'extraAxisOptions',[...
             'ylabel style = {yshift = -1.5em,font = \normalsize},'...
             'xlabel style = {yshift =  0.3em,font = \normalsize},'...
             'xticklabel style = { font = \scriptsize},'...
             'yticklabel style = { font = \scriptsize},'
             ])
end

end

