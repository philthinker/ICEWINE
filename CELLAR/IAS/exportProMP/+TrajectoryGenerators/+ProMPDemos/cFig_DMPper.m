clear variables;
% close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_optWideEnd.mat')

dt           = imData.dataStructure.dt;
numTimeSteps = imData.dataStructure.steps(1).numElements;
Noise_std    = imData.dataStructure.noise_std;
maxAction    = imData.dataStructure.maxAction;
initState    = imData.dataStructure.init_m;
initStateStd = imData.dataStructure.init_std;
numJoints    = length(initState)/2;

%% Linear Env (same as demonstrated system)

settings.setProperty('dt', dt);
settings.setProperty('numTimeSteps', numTimeSteps);
settings.setProperty('Noise_std', Noise_std);
settings.setProperty('maxTorque', maxAction);

sampler = Sampler.EpisodeWithStepsSampler();

dataManager = sampler.getEpisodeDataManager();

environment = Environments.DynamicalSystems.LinearSystem(sampler, numJoints);
sampler.setTransitionFunction(environment);

%% Add perturbation

perturbation = Environments.TransitionFunctionPerturbationConstant(sampler);
sampler.stepSampler.addSamplerFunctionToPool( 'TransitionSampler', 'applyPerturbation', perturbation, -1);


settings.setProperty('useTau', false);
settings.setProperty('numBasis', 35);
settings.setProperty('useWeights', true);

trajectoryGenerator = TrajectoryGenerators.DynamicMovementPrimitives(dataManager, numJoints);
trajectoryGenerator.initObject();
sampler.setParameterPolicy(trajectoryGenerator,'getReferenceTrajectory');

%% PD-Tracker+uff tracker on the ref

%Gains set high enough to match the variance on the via-points
settings.setProperty('PGains', 650 * ones( 1, numJoints ) )
settings.setProperty('DGains',  sqrt(650) * ones( 1, numJoints ) )
settings.setProperty('PDControllerFFGain', 1 * ones( 1, numJoints ) )

ctrTraj = TrajectoryGenerators.TrajectoryTracker.LinearTrajectoryTracker(dataManager, numJoints); 
ctrTraj.initObject();
sampler.setActionPolicy(ctrTraj);

%%  Learner

imitationLearner = TrajectoryGenerators.ImitationLearning.DMPsImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearner.imitationLearningRegularization = 1e-8; 

dataManager.addDataAlias('context',{});
distributionW = Distributions.Gaussian.GaussianLinearInFeatures(dataManager,'Weights','context','GaussianDMPs');
distributionW.initObject();
distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, distributionW);
distributionLearner.minCov = 0.0;

imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);



%% Copy demonstrations
nSamples = imData.dataStructure.numElements; 
trLength = length(imData.dataStructure.steps(1).states);
trData = dataManager.getDataObject([nSamples trLength]);
trData.copyValuesFromDataStructure(imData.dataStructure);

%% Initial State sampling

settings.setProperty('InitialStateDistributionType', 'Gaussian');
settings.setProperty('InitialStateDistributionMinRange', initState-initStateStd);
settings.setProperty('InitialStateDistributionMaxRange', initState+initStateStd);
initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
sampler.setInitialStateSampler(initialStateSampler);

dataManager.finalizeDataManager();

%% Learn
imitationLearnerDistribution.updateModel(trData);

trajectoryGenerator.GoalPos = mean(trData.getDataEntry('jointPositions',:,numTimeSteps));
trajectoryGenerator.GoalVel = mean(trData.getDataEntry('jointVelocities',:,numTimeSteps));


%% Sample trajectories using the ProMP ctl

sampler.numSamples = 500;
sampler.setParallelSampling(true);


perturbation.phaseMax = trajectoryGenerator.phaseGenerator.generatePhaseFromTime(0.2);
perturbation.phaseMin = trajectoryGenerator.phaseGenerator.generatePhaseFromTime(0.3);
perturbation.perturbAmp = -200;

sampleData = dataManager.getDataObject(0);
sampleData.reserveStorage([sampler.numSamples, numTimeSteps]);
sampleData.setDataEntry('Weights',repmat(distributionW.bias',sampler.numSamples,1));
sampler.createSamples(sampleData);

perturbation.perturbAmp = 200;

sampleData1 = dataManager.getDataObject(0);
sampleData1.reserveStorage([sampler.numSamples, numTimeSteps]);
sampleData1.setDataEntry('Weights',repmat(distributionW.bias',sampler.numSamples,1));
sampler.createSamples(sampleData1);

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, figureHandles, {'g'});


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointVelocities', 1, figureHandles, {'g'});


perturbation.phaseMax = trajectoryGenerator.phaseGenerator.generatePhaseFromTime(0.4);
perturbation.phaseMin = trajectoryGenerator.phaseGenerator.generatePhaseFromTime(0.5);
perturbation.perturbAmp = -200;

sampleData = dataManager.getDataObject(0);
sampleData.reserveStorage([sampler.numSamples, numTimeSteps]);
sampleData.setDataEntry('Weights',repmat(distributionW.bias',sampler.numSamples,1));
sampler.createSamples(sampleData);

perturbation.perturbAmp = 200;

sampleData1 = dataManager.getDataObject(0);
sampleData1.reserveStorage([sampler.numSamples, numTimeSteps]);
sampleData1.setDataEntry('Weights',repmat(distributionW.bias',sampler.numSamples,1));
sampler.createSamples(sampleData1);

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, figureHandles, {'g'});


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointVelocities', 1, figureHandles, {'g'});


return

%% Plot 


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles, {'b'});

Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);


Plotter.PlotterData.plotTrajectories(trData,'jointPositions', 1);
Plotter.PlotterData.plotTrajectories(trData,'jointVelocities', 1);

Plotter.PlotterData.plotTrajectories(sampleData,'jointPositions', 1);
Plotter.PlotterData.plotTrajectories(sampleData,'jointVelocities', 1);

%% Reward plotting
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'rewards', 1, [], {'r'});
Plotter.PlotterData.plotTrajectories(sampleData,'rewards', 1);

Plotter.PlotterData.plotTrajectories(sampleData,'returns', 1);
mean(sampleData.getDataEntry('returns'))
std(sampleData.getDataEntry('returns'))/sqrt(sampler.numSamples)

%% Plot Desired vs Training 


f = trajectoryGenerator.plotStateDistribution();
for i=1:numJoints
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', i, f(i), {'r'});
end

f = trajectoryGenerator.plotStateDistribution(1);
for i=1:numJoints
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', i, f(i), {'r'});
end

f = trajectoryGenerator.plotAccDistribution();
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, f(1), {'r'});

figure;
plot(trData.getDataEntry('WeightsImitation')')

x = trData.getDataEntry('WeightsImitation');
figure;plot((x(:,3:end)-2*x(:,2:end-1)+x(:,1:end-2))');

figure;
plot(trData.getDataEntry('basis')')

%% Plot Basis
b=sampleData.getDataEntry('basis',1);
figure;plot(b(:,:))

b=sampleData.getDataEntry('basisD',1);
figure;plot(b(:,:))

%% 
return


TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer1',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer2',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer12',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer3',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer4',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer34',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer1Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer2Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer12Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer3Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer4Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPPer34Vel',1);

