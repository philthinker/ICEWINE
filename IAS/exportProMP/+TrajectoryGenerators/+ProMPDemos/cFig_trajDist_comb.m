clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_pd.mat')

dt           = imData.dataStructure.dt;
numTimeSteps = imData.dataStructure.steps(1).numElements;
Noise_std    = imData.dataStructure.noise_std;
maxAction    = imData.dataStructure.maxAction;
initState    = imData.dataStructure.init_m;
initStateStd = imData.dataStructure.init_std;

%% Linear Env (same as demonstrated system)

settings.setProperty('dt', dt);
settings.setProperty('numTimeSteps', numTimeSteps);
settings.setProperty('Noise_std', Noise_std);
settings.setProperty('maxTorque', maxAction);

sampler = Sampler.EpisodeWithStepsSampler();

dataManager = sampler.getEpisodeDataManager();

numJoints = 1;
environment = Environments.DynamicalSystems.LinearSystem(sampler, numJoints);
% environment.initObject();
sampler.setTransitionFunction(environment);

%% ProMP

settings.setProperty('numBasis', 20);
settings.setProperty('widthFactorBasis', 1.0);
settings.setProperty('numCentersOutsideRange',2)


trajectoryGenerator = TrajectoryGenerators.ProMPs(dataManager, numJoints);
trajectoryGenerator.initObject();

%% ProMP Ctl

settings.setProperty('stochasticCtl',true);
settings.setProperty('estimateNoise',true);

gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, trajectoryGenerator, environment);

ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainGenerator);
sampler.setActionPolicy(ctrTraj);

%% Learner

settings.setProperty('useWeights', true);

distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);
distributionLearner.regularizationRegression = 1e-14; 
distributionLearner.minCov = 0.0;

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-14; 


%% Finilizing setup
sampler.setParameterPolicy(gainGenerator,'updateModel');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');

%% Initial State sampling

if ( 0 ) 
    % Use the learned from ProMPs
    sampler.setInitialStateSampler(trajectoryGenerator);
else
    % or use the same initialization as in the demonstrations    
    settings.setProperty('InitialStateDistributionType', 'Gaussian');
    settings.setProperty('InitialStateDistributionMinRange', initState-initStateStd);
    settings.setProperty('InitialStateDistributionMaxRange', initState+initStateStd);
    initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
    sampler.setInitialStateSampler(initialStateSampler);
end


dataManager.finalizeDataManager();


%% Copy demonstrations
nSamples = imData.dataStructure.numElements; 
trLength = length(imData.dataStructure.steps(1).states);
trData = dataManager.getDataObject([nSamples trLength]);
trData.copyValuesFromDataStructure(imData.dataStructure);

%% Learn
imitationLearnerDistribution.updateModel(trData);


% trajectoryGenerator.conditionTrajectory(0.25, -0.5, 1e-4, [1 0]);
% trajectoryGenerator.conditionTrajectory(0.25, -4, 1e-4, [ 0 1]);
% trajectoryGenerator.conditionTrajectory(0.25, [-0.5 -4]', [1e-6 1e-4]');

%% Sample trajectories using the ProMP ctl
sampler.numSamples = 200;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

tic
sampler.createSamples(sampleData);
toc

%% Plot 


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});
TrajectoryGenerators.ProMPDemos.prepare_plot('asdf',false)


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles, {'b'});

Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);

trajectoryGenerator.plotStateDistribution(sampleData)


