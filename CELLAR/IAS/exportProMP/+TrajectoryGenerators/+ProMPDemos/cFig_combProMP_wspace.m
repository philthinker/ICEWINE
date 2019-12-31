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
settings.setProperty('useJerkPenalty', true);

distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-14;

distributionLearner1 = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator1.distributionW);

imitationLearner1 = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator1, 'jointPositions');
imitationLearnerDistribution1 = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner1, distributionLearner1, trajectoryGenerator1);
                         
imitationLearner1.imitationLearningRegularization = 1e-14;

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

%% Activation Generator

actGen = TrajectoryGenerators.ActivationGenerators.ActivationGenerator(dataManager, 2, 'ALL_ACTIV' );
% actGen = TrajectoryGenerators.ActivationGenerators.ActivationGenerator(dataManager, 2, 'BLEND2' );

%% ProMP Combinator

prompComp = TrajectoryGenerators.ProMPCombiWSpace(dataManager, {trajectoryGenerator, trajectoryGenerator1});

%% ProMP Ctl

if ( 0 )
    settings.setProperty('stochasticCtl',true);
    settings.setProperty('estimateNoise',true);
    
    gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, prompComp, environment);
else    
    settings.setProperty('scaleKp',0.1);
    settings.setProperty('useVelFF',true);
    settings.setProperty('estimateVelFB',false);
    %     settings.setProperty('useFbEigDecomp',true);
    
    gainGenerator = TrajectoryGenerators.CovarianceFbCtl(dataManager, prompComp, environment);
end

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

%% Sample trajectories using the ProMP ctl
sampler.numSamples = 500;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);
sampler.createSamples(sampleData);

mean(sampleData.getDataEntry('returns'))
std(sampleData.getDataEntry('returns'))/sqrt(sampler.numSamples)

actGen.type = 2; % Blend smoothly
sampleData1 = dataManager.getDataObject(0);
sampler.createSamples(sampleData1);

%% Plot 

% OPT
figure;%subplot(2,1,1)
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, gcf, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, gcf, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointPositions', 1, gcf, {'g'});
% subplot(2,1,2)


% Comb
figure;%subplot(2,1,1)
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, gcf, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, gcf, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, gcf, {'g'});
% subplot(2,1,2)
% plot(sampleData.getDataEntry('actvFactor',1,:),'linewidth',2)

% Blend
figure;%subplot(2,1,1)
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, gcf, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointPositions', 1, gcf, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData1,'jointPositions', 1, gcf, {'g'});
% subplot(2,1,2)
% plot(sampleData1.getDataEntry('actvFactor',1,:),'linewidth',2)




return


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'jointVelocities', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'g'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'jointVelocities', 1, figureHandles, {'k'});


[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData1,'actions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles, {'g'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData3,'actions', 1, figureHandles, {'k'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actvFactor', 1, [], {'b'});



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
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptComb',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPComb',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMPBlend',1);
