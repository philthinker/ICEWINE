clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();
settings.setProperty('dt', 0.05);
settings.setProperty('numBasis', 201);
settings.setProperty('numTimeSteps', 200);
settings.setProperty('widthFactorBasis', 5.0);
settings.setProperty('Noise_std', 0.1);
settings.setProperty('numCentersOutsideRange', 0);
settings.setProperty('PGainsDistributionCorrection', [30, 30]);
settings.setProperty('DGainsDistributionCorrection', [6, 6]);
settings.setProperty('correctionTemperature', 2);
settings.setProperty('correctionThreshold', 10);
settings.setProperty('stochasticCtl', 1);
settings.setProperty('useWeights', true);


sampler = Sampler.EpisodeWithStepsSampler();

dataManager = sampler.getEpisodeDataManager();
sampler.getStepSampler.setIsActiveSampler(Sampler.IsActiveStepSampler.IsActiveNumSteps(dataManager));

dimObsState = 2;
dimCtls = 2;

environment = Environments.DynamicalSystems.DoubleLink(sampler);


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

ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, dimCtls, gainGenerator);
%ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearControllerDistributionCorrection(dataManager, trajectoryGenerator, dimCtls, gainGenerator);

sampler.setActionPolicy(ctrTraj);

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner...
                      (dataManager, trajectoryGenerator, 'jointStateAction');
                  
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                                (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-10; 
                         
sampler.setParameterPolicy(gainGenerator,'updateModel');

sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');
%sampler.addParameterPolicy(ctrTraj,'updateModel');

sampler.setInitialStateSampler(trajectoryGenerator);


subDataManager = dataManager.getSubDataManager();
subDataManager.addDataAlias('jointStateAction', 'states', 1:2:4);
subDataManager.addDataAlias('jointStateAction', 'actions');

dataManager.finalizeDataManager();

%%% From file
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_1k.mat'  )
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_1k.mat'  )
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_via_allPerm_9x150.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_via_allPerm_3x150.mat')
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_pd_2link.mat')

numSamplesTraining = 10;
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

savePlot = true;

% Plot --- Desired vs training
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, 1, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, 1, {'b'});

title('');
set(gcf, 'Position', [580 549 643 329]);
xlabel('time(s)', 'FontSize', 20);
ylabel('q(rad)', 'FontSize', 20);
set(gca, 'FontSize', 20);
axis([0 200 -1.8 2])

if (savePlot)
    Plotter.plot2svg('2LinkJoint1', gcf) %Save figure
end



Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 2, 2, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 2, 2, {'b'});

title('');
set(gcf, 'Position', [580 549 643 329]);
xlabel('time(s)', 'FontSize', 20);
ylabel('q(rad)', 'FontSize', 20);
set(gca, 'FontSize', 20);
axis([0 200 -1 1])

if (savePlot)
    Plotter.plot2svg('2LinkJoint2', gcf) %Save figure
end



% % 
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});
