clear variables;
% close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations

load('+TrajectoryGenerators/+ProMPDemos/data/im_data_2d_opt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_2d_opt_smdt.mat')
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_2d_opt_toosmdt.mat')


dt           = imData.dataStructure.dt;
numTimeSteps = imData.dataStructure.steps(1).numElements;
Noise_std    = imData.dataStructure.noise_std;
maxAction    = imData.dataStructure.maxAction;
initState    = imData.dataStructure.init_m;
initStateStd = imData.dataStructure.init_std;
numJoints    = length(initState)/2;

%% Non-Linear Env 

settings.setProperty('dt', dt);
settings.setProperty('numTimeSteps', numTimeSteps);
settings.setProperty('Noise_std', 0*Noise_std);
settings.setProperty('maxTorque', maxAction*1000);

sampler = Sampler.EpisodeWithStepsSampler();

dataManager = sampler.getEpisodeDataManager();


environment = Environments.DynamicalSystems.DoubleLink(sampler);
environment.masses = [3,1];
sampler.setTransitionFunction(environment);

%% ProMP

phaseGenerator = TrajectoryGenerators.PhaseGenerators.PhaseGenerator(dataManager);
if ( 1 )
    %     settings.setProperty('numBasis', 204);
    settings.setProperty('numBasis', 36);
    settings.setProperty('widthFactorBasis', 2);
    settings.setProperty('numCentersOutsideRange',2)
    basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedGaussianBasisGenerator(dataManager, phaseGenerator);
else
    settings.setProperty('numBasis', 50);
    settings.setProperty('bSplineDeg', 5);
    settings.setProperty('bSplineOffsetCoeff', 4);    
    settings.setProperty('normilizeBasis', false);    
    basisGenerator = TrajectoryGenerators.BasisFunctions.NormalizedBSplines(dataManager, phaseGenerator);
end

trajectoryGenerator = TrajectoryGenerators.ProMPs(dataManager, numJoints, phaseGenerator, basisGenerator);
trajectoryGenerator.initObject();

%% ProMP Ctl

settings.setProperty('stochasticCtl',true);
settings.setProperty('estimateNoise',true);
settings.setProperty('linearizeDynamics',true);
settings.setProperty('callPerSample',true);

settings.setProperty('fullNoiseMatrix',true);

gainGenerator = TrajectoryGenerators.ProMPsCtl(dataManager, trajectoryGenerator, environment);

ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, numJoints, gainGenerator);
sampler.setActionPolicy(ctrTraj);

%% Learner

settings.setProperty('useWeights', true);
settings.setProperty('useJerkPenalty', true);

distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);
distributionLearner.minCov = 0.0;
% distributionLearner.regularizationRegression = 1e-4; 
% distributionLearner.minCov = 1e-3; 
% distributionLearner.priorCovWeight =  1e-4;
distributionLearner.maxCorr = 1.0;%0.999;
% distributionLearner.priorCov = 1.0;
% distributionLearner.inputDataNormalization = false;
% distributionLearner.outputDataNormalization = false;

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner(dataManager, trajectoryGenerator, 'jointPositions');
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                             (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-13; 


%% Finilizing setup
% sampler.setParameterPolicy(gainGenerator,'updateModel');
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
elseif( 1 )
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

return

%% Sample trajectories using the ProMP ctl
sampler.numSamples = 100;
sampler.setParallelSampling(true);
sampleData = dataManager.getDataObject(0);

tic
sampler.createSamples(sampleData);
toc

%% Plot 

for i=1:numJoints
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', i, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', i, figureHandles, {'b'});


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', i, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', i, figureHandles, {'b'});

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', i, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', i, figureHandles, {'b'});


end



%% Plot indv
for i=1:numJoints
    
    Plotter.PlotterData.plotTrajectories(sampleData,'actions', i);
    Plotter.PlotterData.plotTrajectories(trData,'actions', i);
    Plotter.PlotterData.plotTrajectories(sampleData,'jointPositions', i);
    Plotter.PlotterData.plotTrajectories(trData,'jointPositions', i);
    Plotter.PlotterData.plotTrajectories(trData,'jointVelocities', i);
    Plotter.PlotterData.plotTrajectories(sampleData,'jointVelocities', i);
    
end

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

%% Plot Basis
b=sampleData.getDataEntry('basis',1);
figure;plot(b(:,:))

b=sampleData.getDataEntry('basisD',1);
figure;plot(b(:,:))

%%

return 

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMP2Link1',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMP2Link1Vel',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMP2Link2',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistProMP2Link2Vel',1);;
