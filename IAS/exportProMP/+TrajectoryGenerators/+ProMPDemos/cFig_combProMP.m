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

%% ProMP

settings.setProperty('numBasis', 35);
settings.setProperty('widthFactorBasis', 2.0);
settings.setProperty('numCentersOutsideRange',2)


trajectoryGenerator = TrajectoryGenerators.ProMPs(dataManager, numJoints);
trajectoryGenerator.initObject();

trajectoryGenerator1 = TrajectoryGenerators.ProMPs(dataManager, numJoints);
trajectoryGenerator1.initObject();

promp_compW = TrajectoryGenerators.ProMPs(dataManager, numJoints);
promp_compW.initObject();

%% Learner

settings.setProperty('useWeights', true);
settings.setProperty('useJerkPenalty', false);

distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);
% distributionLearner.minCov = 0.0;

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-13;
% imitationLearner.imitationLearningRegularization = 1e-6;


distributionLearner1 = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator1.distributionW);
% distributionLearner1.minCov = 0.0;

imitationLearner1 = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator1, 'jointPositions');
imitationLearnerDistribution1 = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner1, distributionLearner1, trajectoryGenerator1);
                         
imitationLearner1.imitationLearningRegularization = 1e-13;
% imitationLearner1.imitationLearningRegularization = 1e-6;

%% Activation Generator

actGen = TrajectoryGenerators.ActivationGenerators.ActivationGenerator(dataManager, 2, 'ALL_ACTIV' );
% actGen = TrajectoryGenerators.ActivationGenerators.ActivationGenerator(dataManager, 2, 'BLEND2' );

%% ProMP Combinator

prompComp = TrajectoryGenerators.ProMPCombi(dataManager, {trajectoryGenerator, trajectoryGenerator1});

%% ProMP Ctl

settings.setProperty('stochasticCtl',true);
settings.setProperty('estimateNoise',true);

gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, prompComp, environment);

ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainGenerator);
sampler.setActionPolicy(ctrTraj);

%% Finilizing setup
sampler.setParameterPolicy(actGen,'generateActivation');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');


%% Initial State sampling

settings.setProperty('InitialStateDistributionType', 'Gaussian');
settings.setProperty('InitialStateDistributionMinRange', initState-initStateStd);
settings.setProperty('InitialStateDistributionMaxRange', initState+initStateStd);
initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
sampler.setInitialStateSampler(initialStateSampler);

dataManager.finalizeDataManager();


%% Copy demonstrations
nSamples = imData.dataStructure.numElements; 
trLength = length(imData.dataStructure.steps(1).states);
trData = dataManager.getDataObject([nSamples trLength]);
trData.copyValuesFromDataStructure(imData.dataStructure);

nSamples = imData2.dataStructure.numElements; 
trLength = length(imData2.dataStructure.steps(1).states);
trData1 = dataManager.getDataObject([nSamples trLength]);
trData1.copyValuesFromDataStructure(imData2.dataStructure);

nSamples = imData3.dataStructure.numElements; 
trLength = length(imData3.dataStructure.steps(1).states);
trData3 = dataManager.getDataObject([nSamples trLength]);
trData3.copyValuesFromDataStructure(imData3.dataStructure);


%% Learn
imitationLearnerDistribution.updateModel(trData);
imitationLearnerDistribution1.updateModel(trData1);

S1 = trajectoryGenerator.distributionW.getCovariance;
m1 = trajectoryGenerator.distributionW.bias;
S2 = trajectoryGenerator1.distributionW.getCovariance;
m2 = trajectoryGenerator1.distributionW.bias;

S3 = S1 + S2;

S = (S1 / S3) * S2;
m = (S1 / ( S1 + S2 ) ) * m2 + (S2 / ( S1 + S2 ) ) * m1;


S22 = inv( inv(S1) + inv(S2) );
m22 = ( inv(S1) + inv(S2) ) \ ( (S1)\m1 + S2\m2);

promp_compW.distributionW.setBias(m);
promp_compW.distributionW.setCovariance(S); 


%% Sample trajectories using the ProMP ctl
sampler.numSamples = 200;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

tic
sampler.createSamples(sampleData);
toc

gainGenerator.trajDistrib = promp_compW;
sampleData1 = dataManager.getDataObject(0);
sampler.createSamples(sampleData1);

%% Plot 


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'g'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, figureHandles, {'y'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointPositions', 1, figureHandles, {'k'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointVelocities', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'g'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointVelocities', 1, figureHandles, {'k'});


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'actions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles, {'g'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'actions', 1, figureHandles, {'k'});
return

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, figureHandles, {'y'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointPositions', 1, figureHandles, {'k'});


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, figureHandles, {'y'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'g'});


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, [], {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'g'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointPositions', 1, figureHandles, {'b'});

Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);
Plotter.PlotterData.plotTrajectories(sampleData1,'actions', 1);

f = trajectoryGenerator.plotStateDistribution()
trajectoryGenerator1.plotStateDistribution(0,f)

f = trajectoryGenerator.plotStateDistribution(1)
trajectoryGenerator1.plotStateDistribution(1,f)


f = prompComp.plotStateDistribution(sampleData)
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'g'});

f = promp_compW.plotStateDistribution();

f = prompComp.plotStateDistribution();

figure;
plot(trData.getDataEntry('WeightsImitation')')

x = trData.getDataEntry('WeightsImitation');
figure;plot((x(:,3:end)-2*x(:,2:end-1)+x(:,1:end-2))');

%% Debug plots

% TrajSpace Desired vs real
f = prompComp.plotStateDistribution();
[~, ~, f] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, f, {'g'});

% TrajSpace Desired vs WSpace Des
f = prompComp.plotStateDistribution();
f = promp_compW.plotStateDistribution(0,f);

% WSpace Des vs real
f = promp_compW.plotStateDistribution(0,f);
[~, ~, f] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, f, {'y'});

% Des: opt, trajspace, wspace
[~, ~, f] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointPositions', 1, [], {'k'});
f = prompComp.plotStateDistribution(0,f,'r');
f = promp_compW.plotStateDistribution(0,f);



%%

return

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPComb',1);
