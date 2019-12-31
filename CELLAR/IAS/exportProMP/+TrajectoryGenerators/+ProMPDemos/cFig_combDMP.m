clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_comb.mat')

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
% environment.initObject();
sampler.setTransitionFunction(environment);

%% DMP Settings

settings.setProperty('useTau', false);
settings.setProperty('numBasis', 35);
settings.setProperty('useWeights', true);
% settings.setProperty('numTimeSteps', 400);

trajectoryGenerator = TrajectoryGenerators.DynamicMovementPrimitives(dataManager, numJoints);
trajectoryGenerator.initObject();
sampler.setParameterPolicy(trajectoryGenerator,'getReferenceTrajectory');

%% PD-Tracker+uff tracker on the ref

%Gains set high enough to match the variance on the via-points
settings.setProperty('PGains', 1000 * ones( 1, numJoints ) )
settings.setProperty('DGains',  31 * ones( 1, numJoints ) )
settings.setProperty('PDControllerFFGain', 1 * ones( 1, numJoints ) )

ctrTraj = TrajectoryGenerators.TrajectoryTracker.LinearTrajectoryTracker(dataManager, numJoints); 
ctrTraj.initObject();
sampler.setActionPolicy(ctrTraj);

%%  Learner

imitationLearner = TrajectoryGenerators.ImitationLearning.DMPsImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearner.imitationLearningRegularization = 1e-8; 

% Common.Settings().setIfEmpty('initSigmaWeights', 10^-9);
dataManager.addDataAlias('context',{});
distributionW = Distributions.Gaussian.GaussianLinearInFeatures(dataManager,'Weights','context','GaussianDMPs');
distributionW.initObject();
distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, distributionW);
distributionLearner.minCov = 0.0;

imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);

%% Reward function

viaPoint.times       = floor([0.20 0.45 0.70 1.00] .*settings.numTimeSteps);
viaPoint.factors     = [10^6, 100; 10^6 100; 10^6 100; 10^6+10^5 100+0.1]*70;
viaPoint.points{1}   = [0.1 0.0];
viaPoint.points{2}   = [-0.1 0.0];
viaPoint.points{3}   = [0.2 0.0 ];
viaPoint.points{4}   = [0.3 0.0];

viaPoint.uFactor = 2*10^-3 / settings.dt;

rewardFunction = RewardFunctions.TimeDependent.ViaPointRewardFunction(dataManager, viaPoint.times,viaPoint.points,viaPoint.factors,viaPoint.uFactor);
sampler.setRewardFunction(rewardFunction);

returnFuncion = RewardFunctions.ReturnForEpisode.ReturnSummedReward(dataManager);
sampler.setReturnFunction(returnFuncion);

%% Initial State

settings.setProperty('InitialStateDistributionType', 'Gaussian');
settings.setProperty('InitialStateDistributionMinRange', initState-initStateStd);
settings.setProperty('InitialStateDistributionMaxRange', initState+initStateStd);
initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
sampler.setInitialStateSampler(initialStateSampler);

dataManager.finalizeDataManager();

%% Copy Imitation data

nSamples = imData.dataStructure.numElements; 
trLength = length(imData.dataStructure.steps(1).states);
trData1 = dataManager.getDataObject([nSamples trLength]);
trData1.copyValuesFromDataStructure(imData.dataStructure);

nSamples = imData2.dataStructure.numElements; 
trLength = length(imData2.dataStructure.steps(1).states);
trData2 = dataManager.getDataObject([nSamples trLength]);
trData2.copyValuesFromDataStructure(imData2.dataStructure);

nSamples = imData.dataStructure.numElements; 
trLength = length(imData.dataStructure.steps(1).states);
trData = dataManager.getDataObject([nSamples,  trLength]);
trData.copyValuesFromDataStructure(imData.dataStructure);
trData.mergeData(trData2);


%% Learn
imitationLearnerDistribution.updateModel(trData);
% imitationLearnerDistribution.updateModel(trData1);
% imitationLearnerDistribution.updateModel(trData2);

%% Sample trajectories using the DMPs ctl

sampler.numSamples = 500;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

sampleData.reserveStorage([sampler.numSamples, numTimeSteps]);
sampleData.setDataEntry('Weights',repmat(distributionW.bias',sampler.numSamples,1));

tic
sampler.createSamples(sampleData);
toc

mean(sampleData.getDataEntry('returns'))
std(sampleData.getDataEntry('returns'))/sqrt(sampler.numSamples)

%% Plotting

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, [], {'b'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData2,'jointPositions', 1, figureHandles, {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, figureHandles, {'k'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'g'});


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointVelocities', 1, [], {'b'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData2,'jointVelocities', 1, figureHandles, {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'g'});

%% 
return

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistDMPComb',1);