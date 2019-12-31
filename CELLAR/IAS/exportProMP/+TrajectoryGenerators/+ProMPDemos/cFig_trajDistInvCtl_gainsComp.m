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

%% Cov Ctl


settings.setProperty('scaleKp',0.4);
settings.setProperty('useVelFF',true);
settings.setProperty('estimateVelFB',false);
%     settings.setProperty('useFbEigDecomp',true);

gainGenerator = TrajectoryGenerators.CovarianceFbCtl(dataManager, trajectoryGenerator, environment);



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


%% Sample trajectories using the ProMP ctl

avg_mean = [];
avg_std = [];
gains = [0.001:0.004:0.05, 0.06:0.01:0.1, 0.2:0.1:2.5];
for i = 1:length(gains)   
    
    settings.setProperty('scaleKp',gains(i));
   
sampler.numSamples = 500;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

sampler.createSamples(sampleData);
avg_mean(i) = mean(sampleData.getDataEntry('returns'));
avg_std(i) = std(sampleData.getDataEntry('returns'));
fprintf('%d/%d Avg ret %f\n',i,length(gains), avg_mean(i));
end

[~, bgains] = max(avg_mean);
gains(bgains)

figure;hold on
idx = [10:23];
plot(gains(idx),avg_mean(idx),'LineWidth',2);
errorbar(gains(idx),avg_mean(idx),avg_std(idx)./sqrt(sampler.numSamples),...
    '.','LineWidth',2,'Color',[0    0.4470    0.7410])

return

%% Actions plot

Plotter.PlotterData.plotTrajectories(sampleData,'actions', 1);
Plotter.PlotterData.plotTrajectories(trData,'actions', 1);

return

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

return
%%

title('')
xlabel('Gain Coeff', 'FontSize', 20);
ylabel('Reward', 'FontSize', 20);
set(gca, 'FontSize', 20);

prefix='+TrajectoryGenerators/+ProMPDemos/+figs/';
fName='InvCtl_GainComp';

Plotter.Matlab2Tikz.matlab2tikz('filename',[prefix,fName,'.tex'],'width','\figwidth',...
             'width','\figheight',...
             'extraAxisOptions',[...
             'ylabel style = {yshift = -1.5em,font = \normalsize},'...
             'xlabel style = {yshift =  0.3em,font = \normalsize},'...
             'xticklabel style = { font = \scriptsize},'...
             'yticklabel style = { font = \scriptsize},'
             ])
