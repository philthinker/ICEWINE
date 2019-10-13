clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();
settings.setProperty('dt', 0.05);
settings.setProperty('numBasis', 201);
% settings.setProperty('numBasis', 40);
settings.setProperty('numTimeSteps', 200);
settings.setProperty('widthFactorBasis', 5.0);
% settings.setProperty('widthFactorBasis', 1.0);
settings.setProperty('Noise_std', 0.3);
settings.setProperty('numCentersOutsideRange', 0);
settings.setProperty('PGainsDistributionCorrection', [50, 40, 30, 20]*3);
% settings.setProperty('DGainsDistributionCorrection', sqrt(0.5*[50, 40, 30, 20]*3));
settings.setProperty('DGainsDistributionCorrection', [3,3, 3, 3]);
settings.setProperty('correctionTemperature', 5);
settings.setProperty('correctionThreshold', 40);
% settings.setProperty('correctionThreshold', 30);
settings.setProperty('stochasticCtl', 0);
settings.setProperty('useWeights', true);


sampler = Sampler.EpisodeWithStepsSampler();

dataManager = sampler.getEpisodeDataManager();
sampler.getStepSampler.setIsActiveSampler(Sampler.IsActiveStepSampler.IsActiveNumSteps(dataManager));

dimObsState = 4;
dimCtls = 4;

environment = Environments.DynamicalSystems.QuadLink(sampler);
environment.friction = ones(1,4) * 5;

dataManager.setRange('actions',-1000*ones(1,dimObsState),1e1000*ones(1,dimObsState));
% dataManager.setRange('actions',-300*ones(1,dimObsState),300*ones(1,dimObsState));
dataManager.setRestrictToRange('actions', false);
environment.initObject();

sampler.setTransitionFunction(environment);

trajectoryGenerator = TrajectoryGenerators.ProMPsModelFree(dataManager, dimObsState, dimCtls);
distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);
distributionLearner.minCov = 0.0;
distributionLearner.maxCorr = 1.0;

gainGenerator = TrajectoryGenerators.ProMPsModelFreeCtl(dataManager, trajectoryGenerator);
%gainGenerator = TrajectoryGenerators.TrajectoryTracker.LearnedLinearFeedbackController(dataManager, {'jointPositions', 'jointVelocities'}, 'actions', environment.dimAction);


trajectoryGenerator.initObject();

%ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, dimCtls, gainGenerator);
ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearControllerDistributionCorrection(dataManager, trajectoryGenerator, dimCtls, gainGenerator);

sampler.setActionPolicy(ctrTraj);

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner...
                      (dataManager, trajectoryGenerator, 'jointStateAction');
                  
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                                (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-10; 
                         
% sampler.addParameterPolicy(gainGenerator,'updateModel');
% sampler.addParameterPolicy(ctrTraj,'updateModel');



sampler.setParameterPolicy(ctrTraj,'updateModel');
sampler.setParameterPolicy(gainGenerator,'updateModel');

sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');







sampler.setInitialStateSampler(trajectoryGenerator);


subDataManager = dataManager.getSubDataManager();
subDataManager.addDataAlias('jointStateAction', 'states', 1:2:8);
subDataManager.addDataAlias('jointStateAction', 'actions');

subDataManager.addDataEntry('cartPos', 2);

dataManager.finalizeDataManager();


% load('+TrajectoryGenerators/+test/+ProMPs/im_data_pd_4link.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_pd_4link_inv.mat')
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_pd_4link_inv_ik.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_pd_4link_inv_ik2.mat')



numSamplesTraining = 50;
dataStructure.numElements = numSamplesTraining;
dataStructure.steps = dataStructure.steps(1:numSamplesTraining);
dataStructure.iterationNumber = dataStructure.iterationNumber(1:numSamplesTraining);

trData = dataManager.getDataObject(0);
trData.copyValuesFromDataStructure(dataStructure);

sampleData = dataManager.getDataObject(0);

nSamples = trData.getNumElements();
% nSamples = 300;
trLength = trData.getNumElementsForDepth(2);


sampler.numSamples = 100;
sampler.setParallelSampling(true);

imitationLearnerDistribution.updateModel(trData);
%gainGenerator.updateModel(trData);
tic
sampler.createSamples(sampleData);
toc

savePlot = false;
setupPlot = false;

% Plot --- Desired vs training
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, 1, {'r'});
ms = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, 1, {'b'});

if ( setupPlot)
    title('');
    set(gcf, 'Position', [580 549 643 329]);
    xlabel('time [s]', 'FontSize', 20);
    ylabel('q [rad]', 'FontSize', 20);
    set(gca, 'FontSize', 20);
    set(gca, 'YTick', [3,4,5] )
    set(gca, 'XTickLabel', num2cell(0:0.5:2))
    axis([0 200 -1 5.3])
end

if (savePlot)
    Plotter.plot2svg('4LinkJoint1', gcf) %Save figure
    matlab2tikz('filename',['4LinkJoint4','.tex'],'width','0.24\linewidth',...
             'extraAxisOptions',[...
             'ylabel style = {yshift = -1.5em},'...
             'xlabel style = {yshift =  0.3em},'...
             'xticklabel style = { },'...
             'yticklabel style = { },'
             ])
end
    



Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 2, 2, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 2, 2, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 3, 3, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 3, 3, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 4, 4, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 4, 4, {'b'});

if ( setupPlot)
    title('');
    set(gcf, 'Position', [580 549 643 329]);
    xlabel('time(s)', 'FontSize', 20);
    ylabel('q(rad)', 'FontSize', 20);
    set(gca, 'XTickLabel', num2cell(0:0.5:2))
    set(gca, 'YTick', [0,1.5,3] )
    set(gca, 'FontSize', 20);
    axis([0 200 0 3])
end

if (savePlot)
    Plotter.plot2svg('4LinkJoint2', gcf) %Save figure
    matlab2tikz('filename',['4LinkJoint2','.tex'],'width','0.75\columnwidth',...
        'extraAxisOptions',[...
        'ylabel style = {yshift = -1.5em,font = \large},'...
        'xlabel style = {yshift =  0.3em,font = \large},'...
        'xticklabel style = { font = \large},'...
        'yticklabel style = { font = \large},'
        ])
end

mx = ms(1:65,:);
environment.animate(ms,2000,20,summer(length(ms)) )


if (savePlot)
    title('');
    axis([-2 4 -0.5 3])
    set(gcf, 'Position', [580 549 643 329]);
    set(gca, 'YTick', [0,1.5,3] )
    set(gca, 'XTick',[-2,0,2,4] )
    Plotter.plot2svg('4LinkJointAnimate', gcf) %Save figure
    matlab2tikz('filename',['4LinkJointAnimate','.tex'],'width','0.75\columnwidth',...
        'extraAxisOptions',[...
        'ylabel style = {yshift = -1.75em},'...
        'xlabel style = {yshift =  0.3em},'...
        'xticklabel style = { },'...
        'yticklabel style = { },'
        ])
end

try
    trData.getDataEntry3D('cartPos');
    
    jointPos = sampleData.getDataEntry3D('jointPositions');
    for i = 1:size(jointPos,1)
        cartPos = environment.getForwardKinematics(squeeze(jointPos(i,:,:)));
        sampleData.setDataEntry('cartPos', cartPos, i);
    end
    
    Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'cartPos', 1, 15, {'r'});
    Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'cartPos', 1, 15, {'b'});
    
    Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'cartPos', 2, 16, {'r'});
    Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'cartPos', 2, 16, {'b'});
    
    if ( setupPlot)
        title('');
        set(gcf, 'Position', [580 549 643 329]);
        xlabel('time(s)', 'FontSize', 20);
        ylabel('y(m)', 'FontSize', 20);
        set(gca, 'FontSize', 20);
        set(gca, 'YTick', [3,4,5] )
        set(gca, 'XTickLabel', num2cell(0:0.5:2))
        axis([0 200 0 3])
    end
    
    if (savePlot)
        Plotter.plot2svg('4LinkCart', gcf) %Save figure
        matlab2tikz('filename',['4LinkCartY','.tex'],'width','0.65\columnwidth',...
            'extraAxisOptions',[...
            'ylabel style = {yshift = -1em},'...
            'xlabel style = {yshift =  0.3em},'...
            'xticklabel style = { },'...
            'yticklabel style = { },'
            ])
    end
    
    
    
catch e
    display('No Cart data in training')
end

 Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'activationPD', 1, 20, {'r'});
 
% trPos = trData.getDataEntry3D('jointPositions');
% for j = 1:4
%     figure;hold on
%     for i = 1:size(jointPos,1)
%         plot(squeeze(jointPos(i,:,j)),'b')
%     end
%     for i = 1:size(trPos,1)
%         plot(squeeze(trPos(i,:,j)),'r')
%     end
% end
 
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions',1, figure, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions',2, figure, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions',3, figure, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions',4, figure, {'b'});


Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions',1, figure, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions',2, figure, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions',3, figure, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions',4, figure, {'b'});
 

% % 
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});
