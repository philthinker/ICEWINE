clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_optWideEnd.mat')

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

%% ProMP/Cov Ctl

if ( 1 )
    settings.setProperty('stochasticCtl',true);
    settings.setProperty('estimateNoise',true);
    
    gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, trajectoryGenerator, environment);
else    
    settings.setProperty('scaleKp',0.4);
    settings.setProperty('scaleKd',0.1);
    settings.setProperty('useVelFF',true);
    settings.setProperty('estimateVelFB',false);
    %     settings.setProperty('useFbEigDecomp',true);
    
    gainGenerator = TrajectoryGenerators.CovarianceFbCtl(dataManager, trajectoryGenerator, environment);
end


ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainGenerator);
sampler.setActionPolicy(ctrTraj);

%% Learner

settings.setProperty('useWeights', true);
settings.setProperty('useJerkPenalty', true);

distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);
% distributionLearner.regularizationRegression = 1e-4; 
% distributionLearner.minCov = 1e-3; 
% distributionLearner.priorCovWeight =  1e-3;
distributionLearner.maxCorr = 1.0;
% distributionLearner.priorCov = 1.0;
% distributionLearner.inputDataNormalization = false;
% distributionLearner.outputDataNormalization = false;

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-12; 


%% Copy demonstrations
nSamples = imData.dataStructure.numElements; 
trLength = length(imData.dataStructure.steps(1).states);
trData = dataManager.getDataObject([nSamples trLength]);
trData.copyValuesFromDataStructure(imData.dataStructure);

%% Initial State sampling

if ( 0 ) 
    % Use the learned from ProMPs
    sampler.setInitialStateSampler(trajectoryGenerator);
elseif ( 1 )
    % or use the same initialization as in the demonstrations    
    settings.setProperty('InitialStateDistributionType', 'Gaussian');
    settings.setProperty('InitialStateDistributionMinRange', initState-initStateStd);
    settings.setProperty('InitialStateDistributionMaxRange', initState+initStateStd);
    initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
    sampler.setInitialStateSampler(initialStateSampler);
else
    settings.setProperty('useLastState', 'false');
    initialStateSampler = Sampler.InitialSampler.InitialStateSamplerFromTrainingData(sampler,trData);
    sampler.setInitialStateSampler(initialStateSampler);       
end


dataManager.finalizeDataManager();

%% Learn
imitationLearnerDistribution.updateModel(trData);

%% Change the speed of the phase

constSpeed = 1.333333333333;
oldSteps = numTimeSteps;
numTimeSteps = round(oldSteps / constSpeed);
settings.setProperty('numTimeSteps', numTimeSteps);
sp = constSpeed * ones(1,numTimeSteps);
phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGeneratorSpeedProfile(dataManager,sp);

trajectoryGenerator.phaseGenerator = phaseGenerator;

sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');


%% Sample trajectories using the ProMP ctl

sampler.numSamples = 500;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

sampler.createSamples(sampleData);

%% Make it faster
constSpeed = 0.8;
numTimeSteps = round(oldSteps / constSpeed);
settings.setProperty('numTimeSteps', numTimeSteps);
sp = constSpeed * ones(1,numTimeSteps);
phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGeneratorSpeedProfile(dataManager,sp);

trajectoryGenerator.phaseGenerator = phaseGenerator;

sampler.addSamplerFunctionToPool( 'ParameterPolicy', 'generateBasisDD', trajectoryGenerator.basisGenerator,  0);
sampler.addSamplerFunctionToPool( 'ParameterPolicy', 'generateBasisD',  trajectoryGenerator.basisGenerator, -1);
sampler.addSamplerFunctionToPool( 'ParameterPolicy', 'generatePhaseDD', trajectoryGenerator.phaseGenerator, -1);
sampler.addSamplerFunctionToPool( 'ParameterPolicy', 'generatePhaseD',  trajectoryGenerator.phaseGenerator, -1);

sampleData1 = dataManager.getDataObject(0);
sampler.createSamples(sampleData1);

%% Plot 


Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, gcf, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, gcf, {'g'});


Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, gcf, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointVelocities', 1, gcf, {'g'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, gcf, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'actions', 1, gcf, {'g'});

% Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);

f = trajectoryGenerator.plotStateDistribution();
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, f, {'r'});
f = trajectoryGenerator.plotStateDistribution(1);
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, f, {'r'});

%% Save plots

return 

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPSpeed',1);

