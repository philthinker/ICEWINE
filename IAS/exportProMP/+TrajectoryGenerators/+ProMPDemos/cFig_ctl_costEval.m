function [cost_m, cost_std] =  cFig_ctl_costEval(numBasis, useProMP, numDemo)

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


%% ProMP

phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);

settings.setProperty('numBasis', numBasis);
settings.setProperty('widthFactorBasis', 1.0);
settings.setProperty('numCentersOutsideRange',2)
basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedGaussianBasisGenerator(dataManager, phaseGenerator);


trajectoryGenerator = TrajectoryGenerators.ProMPs(dataManager, numJoints, phaseGenerator, basisGenerator);
trajectoryGenerator.initObject();

%% ProMP/Cov Ctl

if ( useProMP )
    settings.setProperty('stochasticCtl',true);
    settings.setProperty('estimateNoise',true);
    
    gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, trajectoryGenerator, environment);
else    
    settings.setProperty('scaleKp',0.4);
    settings.setProperty('useVelFF',true);
    settings.setProperty('estimateVelFB',false);
    %     settings.setProperty('useFbEigDecomp',true);
    
    gainGenerator = TrajectoryGenerators.CovarianceFbCtl(dataManager, trajectoryGenerator, environment);
end


ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainGenerator);
sampler.setActionPolicy(ctrTraj);

%% Reward function --- only for comparison to the opt controller

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
phaseGenerator.callDataFunction('generatePhaseD',trData);
phaseGenerator.callDataFunction('generatePhaseDD',trData);
basisGenerator.callDataFunction('generateBasisD',trData);
basisGenerator.callDataFunction('generateBasisDD',trData);
imitationLearnerDistribution.updateModel(trData);


%% Sample trajectories using the ProMP ctl

sampler.numSamples = 500;
sampler.setParallelSampling(true);



imitationLearnerDistribution.updateModel(trData);
sampleData = dataManager.getDataObject(0);
sampler.createSamples(sampleData);
cost_m = mean(sampleData.getDataEntry('returns'));
cost_std = std(sampleData.getDataEntry('returns'))/sqrt(sampler.numSamples);

