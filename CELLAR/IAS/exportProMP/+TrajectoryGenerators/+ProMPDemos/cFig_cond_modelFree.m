clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();
settings.setProperty('dt', 0.005);
settings.setProperty('numBasis', 50);
settings.setProperty('numTimeSteps', 200);
settings.setProperty('widthFactorBasis', 1.5);
settings.setProperty('Noise_std', 0.5);
settings.setProperty('useWeights', true);
% settings.setProperty('Noise_std', 0);


sampler = Sampler.EpisodeWithStepsSampler();

dataManager = sampler.getEpisodeDataManager();
sampler.getStepSampler.setIsActiveSampler(Sampler.IsActiveStepSampler.IsActiveNumSteps(dataManager));

dimObsState = 1;
dimCtls = 1;

environment = Environments.DynamicalSystems.LinearSystem(sampler, dimObsState);
environment.masses = 1;

dataManager.setRange('actions',-1e10*ones(1,dimObsState),1e10*ones(1,dimObsState));
% dataManager.setRange('actions',-300*ones(1,dimObsState),300*ones(1,dimObsState));
dataManager.setRestrictToRange('actions', false);
environment.initObject();

sampler.setTransitionFunction(environment);

trajectoryGenerator = TrajectoryGenerators.ProMPsModelFree(dataManager, dimObsState, dimCtls);
distributionLearner = Learner.SupervisedLearner.LinearGaussianMLLearner(dataManager, trajectoryGenerator.distributionW);
distributionLearner.minCov = 0.0;
distributionLearner.maxCorr = 1.0;

gainGenerator = TrajectoryGenerators.ProMPsModelFreeCtl(dataManager, trajectoryGenerator);

trajectoryGenerator.initObject();

% sampler.addParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');
% sampler.addParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
% sampler.addParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
% sampler.addParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');

ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, dimCtls, gainGenerator);
sampler.setActionPolicy(ctrTraj);

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner...
                      (dataManager, trajectoryGenerator, 'jointStateAction');
                  
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                                (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-18;
                         


sampler.setInitialStateSampler(trajectoryGenerator);


dataManager.addDataEntry('steps.jointStateAction', dimObsState+dimCtls);

sampler.setParameterPolicy(gainGenerator,'updateModel');

sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');




dataManager.finalizeDataManager();

%%% From file
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_1k.mat'  )
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_1k.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_20k.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_via_allPerm_9x150.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_via_allPerm_3x150.mat')
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_pd_1k.mat')
nSamples = length(imlearn.q);
% nSamples = 300;
trLength = size(imlearn.q{1},1);

trData = dataManager.getDataObject([nSamples trLength]);
sampleData = dataManager.getDataObject(0);


useNoise = false;

figure(100);hold all;
figure(101);hold all;
for i = 1:nSamples
    figure(100);plot(imlearn.q{i}(:,1));
    if (useNoise)
        imlearn.uNoise{i}(end+1,:) = imlearn.uNoise{i}(end,:);
        figure(101);plot(imlearn.uNoise{i});
        trData.setDataEntry('jointStateAction', ...
            [imlearn.q{i}(:,1:2:end) , imlearn.uNoise{i}], i);        
         trData.setDataEntry('actions', imlearn.uNoise{i}, i);
    else
        d = imlearn.u{i}(end,:) - imlearn.u{i}(end-1,:);
        imlearn.u{i}(end+1,:) = imlearn.u{i}(end,:)+d;
        figure(101);plot(imlearn.u{i});
        trData.setDataEntry('jointStateAction', ...
            [imlearn.q{i}(:,1:2:end) , imlearn.u{i}], i);
        trData.setDataEntry('actions', imlearn.u{i}, i);
    end

                     
    trData.setDataEntry('jointPositions', imlearn.q{i}(:,1:2:end), i);  %for comp plotting
    trData.setDataEntry('jointVelocities', imlearn.q{i}(:,2:2:end), i);
end

imitationLearnerDistribution.updateModel(trData);

figureHandlesBefB = trajectoryGenerator.plotStateDistribution(0,[],'r');
figureHandlesBefB2 = trajectoryGenerator.plotStateDistribution(0,[],'r');
figureHandlesBefB3 = trajectoryGenerator.plotStateDistribution(0,[],'r');

% trajectoryGenerator.conditionTrajectory( 0.25, -0.5, 1e-6, [1 0 0 0]);
% trajectoryGenerator.conditionTrajectory( 0.25, -4, 1e-4, [ 0 1]);
% trajectoryGenerator.conditionTrajectory( 0.25, [-0.5 -4]', [1e-6 1e-4]');

% trajectoryGenerator.conditionTrajectory( 0.75, 1, 1e-6, [1 0 0 0]);
% trajectoryGenerator.conditionTrajectory( 0.75, 0.85, 1e-6, [1 0 0 0]);
% trajectoryGenerator.conditionTrajectory( 0.25, -0.5, 1e-6, [1 0 0 0]);

trajectoryGenerator.conditionTrajectory( 0.75, 0.5, 1e-4, [1 0 0 0]);

sampler.numSamples = 100;
sampler.setParallelSampling(true);

tic
sampler.createSamples(sampleData);
toc

% trajectoryGenerator.plotStateDistribution(trData)

%%

% Plot --- Desired before after cond
trajectoryGenerator.plotStateDistribution(0,figureHandlesBefB,'b');
figure(figureHandlesBefB(1));title('Desired before vs after cond Pos')
figure(figureHandlesBefB(2));title('Desired before vs after cond Act')

% Plot --- Desired before vs training
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, figureHandlesBefB2, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, figureHandlesBefB2(2), {'b'});
figure(figureHandlesBefB2(1));title('Desired before vs training Pos')
figure(figureHandlesBefB2(2));title('Desired before vs training Act')


% Plot --- Desired after vs reproduction
figureHandles = trajectoryGenerator.plotStateDistribution(0,[],'r');
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles(2), {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});
figure(figureHandles(1));title('Desired after vs reproduction Pos')
figure(figureHandles(2));title('Desired after vs reproduction Act')

% Plot --- Deisred before vs reproduction
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandlesBefB3(2), {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandlesBefB3, {'b'});
figure(figureHandlesBefB3(1));title('Deisred before vs reproduction Pos')
figure(figureHandlesBefB3(2));title('Deisred before vs reproduction Act')

% 
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles(2), {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, figureHandles, {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});



