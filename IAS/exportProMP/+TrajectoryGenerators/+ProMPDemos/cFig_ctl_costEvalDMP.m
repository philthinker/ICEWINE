function [cost_m, cost_std] =  cFig_ctl_costEvalDMP(numBasis, numDemo)

settings = Common.Settings();
settings.clean();

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

settings.setProperty('useTau', false);
settings.setProperty('numBasis', numBasis);
settings.setProperty('useWeights', true);

trajectoryGenerator = TrajectoryGenerators.DynamicMovementPrimitives(dataManager, numJoints);
trajectoryGenerator.initObject();
sampler.setParameterPolicy(trajectoryGenerator,'getReferenceTrajectory');

%% PD-Tracker+uff tracker on the ref

%Gains set high enough to match the variance on the via-points
% settings.setProperty('PGains', 1000 * ones( 1, numJoints ) )
% settings.setProperty('DGains',  sqrt(1000) * ones( 1, numJoints ) )
settings.setProperty('PGains', 650 * ones( 1, numJoints ) )
settings.setProperty('DGains',  sqrt(650) * ones( 1, numJoints ) )
% settings.setProperty('PGains', 50 * ones( 1, numJoints ) )
% settings.setProperty('DGains',  sqrt(50) * ones( 1, numJoints ) )
settings.setProperty('PDControllerFFGain', 1 * ones( 1, numJoints ) )

ctrTraj = TrajectoryGenerators.TrajectoryTracker.LinearTrajectoryTracker(dataManager, numJoints); 
ctrTraj.initObject();
sampler.setActionPolicy(ctrTraj);

%% Reward function

viaPoint.times       = floor([0.40 0.70 1.0] .*settings.numTimeSteps);
viaPoint.factors     = [10^6, 1; 10^5 10^3; 10^1 1]*70;
viaPoint.points{1}   = [-0.7 0.0];
viaPoint.points{2}   = [0.0 0 ];
viaPoint.points{3}   = [0.0 0.0 ];

viaPoint.uFactor = 10^-3 / settings.dt;

rewardFunction = RewardFunctions.TimeDependent.ViaPointRewardFunction(dataManager, viaPoint.times,viaPoint.points,viaPoint.factors,viaPoint.uFactor);
sampler.setRewardFunction(rewardFunction);

returnFuncion = RewardFunctions.ReturnForEpisode.ReturnSummedReward(dataManager);
sampler.setReturnFunction(returnFuncion);

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
if ( ~exist('numDemo','var') || isempty(numDemo) )
    nSamples = imData.dataStructure.numElements;
    trLength = length(imData.dataStructure.steps(1).states);
    trData = dataManager.getDataObject([nSamples trLength]);
    trData.copyValuesFromDataStructure(imData.dataStructure);
else
    trLength = length(imData.dataStructure.steps(1).states);
    trData = dataManager.getDataObject([numDemo trLength]);
    trData.copyValuesFromDataStructure(imData.dataStructure);
    trData.deleteData(randperm(imData.dataStructure.numElements,numDemo));
end


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
sampleData = dataManager.getDataObject(0);

sampleData.reserveStorage([sampler.numSamples, numTimeSteps]);
sampleData.setDataEntry('Weights',repmat(distributionW.bias',sampler.numSamples,1));

sampler.createSamples(sampleData);

cost_m = mean(sampleData.getDataEntry('returns'));
cost_std = std(sampleData.getDataEntry('returns'))/sqrt(sampler.numSamples);

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

