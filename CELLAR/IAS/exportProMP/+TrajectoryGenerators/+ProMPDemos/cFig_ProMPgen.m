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


%% ProMP

phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);

settings.setProperty('numBasis', 35);
settings.setProperty('widthFactorBasis', 1.0);
settings.setProperty('numCentersOutsideRange',2)
basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedGaussianBasisGenerator(dataManager, phaseGenerator);


trajectoryGenerator = TrajectoryGenerators.ProMPs(dataManager, numJoints, phaseGenerator, basisGenerator);
trajectoryGenerator.initObject();

%% ProMP
settings.setProperty('stochasticCtl',false);
settings.setProperty('estimateNoise',true);

gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, trajectoryGenerator, environment);

ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainGenerator);
sampler.setActionPolicy(ctrTraj);

%% Learner

settings.setProperty('useWeights', true);
settings.setProperty('useJerkPenalty', true);

distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
    (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);

imitationLearner.imitationLearningRegularization = 1e-13;
% imitationLearner.imitationLearningRegularization = 1e-6;

%% Finilizing setup
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');

%% Copy demonstrations
nSamples = imData.dataStructure.numElements;
trLength = length(imData.dataStructure.steps(1).states);
trData = dataManager.getDataObject([nSamples trLength]);
trData.copyValuesFromDataStructure(imData.dataStructure);

%% Initial State sampling

% or use the same initialization as in the demonstrations
settings.setProperty('InitialStateDistributionType', 'Gaussian');
settings.setProperty('InitialStateDistributionMinRange', initState-initStateStd);
settings.setProperty('InitialStateDistributionMaxRange', initState+initStateStd);
initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
sampler.setInitialStateSampler(initialStateSampler);

dataManager.finalizeDataManager();

%% Learn
phaseGenerator.callDataFunction('generatePhaseD',trData);
phaseGenerator.callDataFunction('generatePhaseDD',trData);
basisGenerator.callDataFunction('generateBasisD',trData);
basisGenerator.callDataFunction('generateBasisDD',trData);
imitationLearnerDistribution.updateModel(trData);

sampler.numSamples = 500;
sampler.setParallelSampling(true);

return

%% Sample trajectories using the ProMP ctl

sampleData = dataManager.getDataObject(0);
sampler.createSamples(sampleData);

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});

%% Conditioning Traj Gen

trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory(1.0, 0.2, 1e-6, [1 0]);

sampleData1 = dataManager.getDataObject(0);
sampler.createSamples(sampleData1);

trajectoryGenerator.pop();
trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory(1.0, -0.2, 1e-6, [1 0]);

sampleData2 = dataManager.getDataObject(0);
sampler.createSamples(sampleData2);

trajectoryGenerator.pop();

%% Plotting Traj Gen

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData2,'jointPositions', 1, figureHandles, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, figureHandles, {'g'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData2,'jointVelocities', 1, figureHandles, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointVelocities', 1, figureHandles, {'g'});

%% Conditioning Traj Gen

trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory(1.0, 1.5, 1e-3, [0 1]);

sampleData3 = dataManager.getDataObject(0);
sampler.createSamples(sampleData3);

trajectoryGenerator.pop();
trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory(1.0, -1.5, 1e-3, [0 1]);

sampleData4 = dataManager.getDataObject(0);
sampler.createSamples(sampleData4);

trajectoryGenerator.pop();

%% Plotting Traj Gen

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData4,'jointPositions', 1, figureHandles, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData3,'jointPositions', 1, figureHandles, {'g'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData4,'jointVelocities', 1, figureHandles, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData3,'jointVelocities', 1, figureHandles, {'g'});

%% Conditioning Traj Gen

trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory(1.0, [0.1; 2.5], [1e-4; 1e-2]);
trajectoryGenerator.conditionTrajectory(0.25, [-0.45; -3.0 ],[1e-4; 1e-2]);

sampleData5 = dataManager.getDataObject(0);
sampler.createSamples(sampleData5);

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData5,'jointPositions', 1, figureHandles, {'b'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData5,'jointVelocities', 1, figureHandles, {'b'});

Plotter.PlotterData.plotTrajectories(sampleData5,'actions', 1);

trajectoryGenerator.plotStateDistribution();
trajectoryGenerator.plotStateDistribution(1);

trajectoryGenerator.pop();
return

%% Actions plot

Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);
Plotter.PlotterData.plotTrajectories(trData,'actions', 1);


%% Plot

Plotter.PlotterData.plotTrajectories(trData,'jointPositions', 1);
Plotter.PlotterData.plotTrajectories(trData,'jointVelocities', 1);

Plotter.PlotterData.plotTrajectories(sampleData,'jointPositions', 1);
Plotter.PlotterData.plotTrajectories(sampleData,'jointVelocities', 1);


%%
return

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOpt',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptVel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistPromp',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistPrompVel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistCovCtl',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistCovCtlVel',1);


TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCond1',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCond2',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCond12',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCond1Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCond2Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCond12Vel',1);


TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondVel1',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondVel2',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondVel12',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondVel1Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondVel2Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondVel12Vel',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondBoth',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPCondBothVel',1);


