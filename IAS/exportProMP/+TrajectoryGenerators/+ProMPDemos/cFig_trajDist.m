clear variables;
% close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_optRecInit.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt2K.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_2d_opt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_2d_opt_smdt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_pd.mat')
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

if ( 1 )
        settings.setProperty('phaseMin', 0.2);
        settings.setProperty('phaseMax', 0.3);
%     settings.setProperty('phaseMin', 0.4);
%     settings.setProperty('phaseMax', 0.5);
%     settings.setProperty('perturbAmp', -200);
    settings.setProperty('perturbAmp', 200);
    
    perturbation = Environments.TransitionFunctionPerturbationConstant(sampler);
    sampler.stepSampler.addSamplerFunctionToPool( 'TransitionSampler', 'applyPerturbation', perturbation, -1);
end

%% ProMP

phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);
if ( 1 )
    settings.setProperty('numBasis', 35);
    settings.setProperty('widthFactorBasis', 1.0);
    settings.setProperty('numCentersOutsideRange',2)
    basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedGaussianBasisGenerator(dataManager, phaseGenerator);
else
    settings.setProperty('numBasis', 25);
    settings.setProperty('bSplineDeg', 5);
    settings.setProperty('bSplineOffsetCoeff', 5);    
    settings.setProperty('normilizeBasis', false);    
    basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedBSplines(dataManager, phaseGenerator);
end

trajectoryGenerator = TrajectoryGenerators.ProMPs(dataManager, numJoints, phaseGenerator, basisGenerator);
trajectoryGenerator.initObject();

%% ProMP/Cov Ctl

if ( 1 )
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
% viaPoint.factors     = [10^5, 1; 10^6 1; 10^5 10^2]*70;
viaPoint.factors     = [10^6, 1; 10^5 10^3; 10^1 1]*70;
viaPoint.points{1}   = [-0.7 0.0];
% viaPoint.points{2}   = [0.8 0.0 ];
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
% distributionLearner.regularizationRegression = 1e-4; 
% distributionLearner.minCov = 0*1e-3; 
% distributionLearner.priorCovWeight =  1e-18;
distributionLearner.maxCorr = 1.0;
% distributionLearner.priorCov = 1.0;
% distributionLearner.inputDataNormalization = false;
% distributionLearner.outputDataNormalization = false;

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
    settings.setProperty('useLastState', false);
    initialStateSampler = Sampler.InitialSampler.InitialStateSamplerFromTrainingData(sampler,trData);
    sampler.setInitialStateSampler(initialStateSampler);       
end


dataManager.finalizeDataManager();

%% Learn
phaseGenerator.callDataFunction('generatePhaseD',trData);
phaseGenerator.callDataFunction('generatePhaseDD',trData);
basisGenerator.callDataFunction('generateBasisD',trData);
basisGenerator.callDataFunction('generateBasisDD',trData);
imitationLearnerDistribution.updateModel(trData);

%% Conditioning
% trajectoryGenerator.conditionTrajectory(1.0, 0, 1e-6, [1 0]);
% trajectoryGenerator.conditionTrajectory(1.0, 0.2, 1e-6, [1 0]);
% trajectoryGenerator.conditionTrajectory(1.0, -0.2, 1e-6, [1 0]);

% trajectoryGenerator.conditionTrajectory(0.5, -0.55, 1e-5, [1 0]);
% trajectoryGenerator.conditionTrajectory(0.25, -0.4, 1e-5, [1 0]);
% trajectoryGenerator.conditionTrajectory(1, 0.0, 2e-1, [1 0]);


% trajectoryGenerator.conditionTrajectory(1.0, -1.5, 1e-3, [0 1]);
% trajectoryGenerator.conditionTrajectory(1.0, 1.5, 1e-3, [0 1]);
% trajectoryGenerator.conditionTrajectory(0.25, -3, 1e-3, [0 1]);
% trajectoryGenerator.conditionTrajectory(0.5, 3.5, 1e-3, [0 1]);


% trajectoryGenerator.conditionTrajectory(0.25, -4, 1e-4, [ 0 1]);
% trajectoryGenerator.conditionTrajectory(0.25, [-0.5 -4]', [1e-6 1e-4]');

%% Sample trajectories using the ProMP ctl

sampler.numSamples = 500;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

sampler.createSamples(sampleData);
return

%% Actions plot

Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);
Plotter.PlotterData.plotTrajectories(trData,'actions', 1);


%% Plot 

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles, {'b'});

f = trajectoryGenerator.plotStateDistribution();
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, f, {'r'});
f = trajectoryGenerator.plotStateDistribution(1);
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, f, {'r'});

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


TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer1',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer2',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer12',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer3',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer4',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer34',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer1Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer2Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer12Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer3Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer4Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPPer34Vel',1);
