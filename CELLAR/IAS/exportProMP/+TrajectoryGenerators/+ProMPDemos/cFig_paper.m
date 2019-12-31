clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();
settings.setProperty('dt', 0.005);
settings.setProperty('numBasis', 80);
settings.setProperty('numTimeSteps', 200);
settings.setProperty('widthFactorBasis', 1.4);
settings.setProperty('Noise_std', 0.5);
settings.setProperty('useWeights', true);

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



ctrTraj = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, dimCtls, gainGenerator);
sampler.setActionPolicy(ctrTraj);

imitationLearner = TrajectoryGenerators.ImitationLearning.LinearTrajectoryImitationLearner...
                      (dataManager, trajectoryGenerator, 'jointStateAction');
                  
imitationLearnerDistribution = TrajectoryGenerators.ImitationLearning.ParameterDistributionImitationLearner...
                                (dataManager, imitationLearner, distributionLearner, trajectoryGenerator);
                         
imitationLearner.imitationLearningRegularization = 1e-18;

sampler.setInitialStateSampler(trajectoryGenerator);
                         
sampler.setParameterPolicy(gainGenerator,'updateModel');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');



dataManager.addDataEntry('steps.jointStateAction', dimObsState+dimCtls);

dataManager.finalizeDataManager();

%%% From file
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_1k.mat'  )
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_1k.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_20k.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_via_allPerm_9x150.mat')
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_via_allPerm_3x150.mat')
load('+TrajectoryGenerators/+ProMPDemos/data/im_data_pd_1k.mat')
nSamples = length(imlearn.q);
nSamples = 100;
trLength = size(imlearn.q{1},1);

trData = dataManager.getDataObject([nSamples trLength]);




for i = 1:nSamples
    d = imlearn.u{i}(end,:) - imlearn.u{i}(end-1,:);
    imlearn.u{i}(end+1,:) = imlearn.u{i}(end,:)+d;
    trData.setDataEntry('jointStateAction', ...
        [imlearn.q{i}(:,1:2:end) , imlearn.u{i}], i);
    trData.setDataEntry('actions', imlearn.u{i}, i);
    
    trData.setDataEntry('jointPositions', imlearn.q{i}(:,1:2:end), i);  %for comp plotting
    trData.setDataEntry('jointVelocities', imlearn.q{i}(:,2:2:end), i);
end

imitationLearnerDistribution.updateModel(trData);

figureHandlesBefB = trajectoryGenerator.plotStateDistribution(0,[],'r');
figureHandlesBefB2 = trajectoryGenerator.plotStateDistribution(0,[],'r');
figureHandlesBefB3 = trajectoryGenerator.plotStateDistribution(0,[],'r');

figureHandlesBefB1V = trajectoryGenerator.plotStateDistribution(1,[],'r');

% trajectoryGenerator.conditionTrajectory( 0.25, -0.5, 1e-6, [1 0 0 0]);
% trajectoryGenerator.conditionTrajectory( 0.25, -4, 1e-4, [ 0 1]);
% trajectoryGenerator.conditionTrajectory( 0.25, [-0.5 -4]', [1e-6 1e-4]');

% trajectoryGenerator.conditionTrajectory( 0.75, 1, 1e-6, [1 0 0 0]);
% trajectoryGenerator.conditionTrajectory( 0.75, 0.85, 1e-6, [1 0 0 0]);
% trajectoryGenerator.conditionTrajectory( 0.25, -0.5, 1e-6, [1 0 0 0]);

sampler.numSamples = 500;
sampler.setParallelSampling(true);

noCond = dataManager.getDataObject(0);
tic; sampler.createSamples(noCond); toc;

%%%%%%%%%%%%%%
trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory( 0.75, 1.3, 1e-4, [1 0 0 0]);

cond1 = dataManager.getDataObject(0);
tic; sampler.createSamples(cond1); toc;

trajectoryGenerator.pop();

%%%%%%%%%%%%%%
trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory( 0.75, 0.8, 1e-4, [1 0 0 0]);

cond2 = dataManager.getDataObject(0);
tic; sampler.createSamples(cond2); toc;

trajectoryGenerator.pop();

%%%%%%%%%%%%%%
trajectoryGenerator.push();
trajectoryGenerator.conditionTrajectory( 0.75, 0.5, 1e-4, [1 0 0 0]);

cond3 = dataManager.getDataObject(0);
tic; sampler.createSamples(cond3); toc;

trajectoryGenerator.pop();

%%

savePlot = true;

% Plot --- Desired before vs reprod before

Plotter.PlotterData.plotTrajectoriesMeanAndStd(noCond,'actions', 1, figureHandlesBefB(2), {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(noCond,'jointPositions', 1, figureHandlesBefB, {'b'});
figure(figureHandlesBefB(1));
TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodJoints', savePlot);
title('Desired vs reprod Pos')
figure(figureHandlesBefB(2));
TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodActions', savePlot);
title('Desired vs reprod Act')

% Plot --- Desired after vs reproduction
figureHandles = trajectoryGenerator.plotStateDistribution(0,[],'r');
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond1,'actions', 1, figureHandles(2), {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond1,'jointPositions', 1, figureHandles, {'b'});
figure(figureHandles(1));
TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodJoints1', savePlot);
title('Desired after vs reproduction Pos')
figure(figureHandles(2));
TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodActions1', savePlot);
title('Desired after vs reproduction Act')

% Plot --- Desired after vs reproduction
figureHandles = trajectoryGenerator.plotStateDistribution(0,[],'r');
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond2,'actions', 1, figureHandles(2), {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond2,'jointPositions', 1, figureHandles, {'b'});
figure(figureHandles(1));
TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodJoints2', savePlot);
title('Desired after vs reproduction Pos')
figure(figureHandles(2));
TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodActions2.pdf', savePlot);
title('Desired after vs reproduction Act')

[~, ~, figureHandles] = ...
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond1,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond2,'jointPositions', 1, figureHandles, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond3,'jointPositions', 1, figureHandles, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(noCond,'jointPositions', 1, figureHandles, {'b'});

plot(0.75 * 200, 0.5, 'rx', 'MarkerSize', 8, 'LineWidth',2);
plot(0.75 * 200, 0.8, 'rx', 'MarkerSize', 8, 'LineWidth',2);
plot(0.75 * 200, 1.3, 'rx', 'MarkerSize', 8, 'LineWidth',2);


TrajectoryGenerators.test.ProMPs.prepare_plot('DesiredReprodJointsAll', savePlot);
title('Desired after vs reproduction Pos')



return

% Plot --- Desired before after cond
trajectoryGenerator.plotStateDistribution(0,figureHandlesBefB,'b');
figure(figureHandlesBefB(1));
prepare_plot();
title('Desired before vs after cond Pos')
figure(figureHandlesBefB(2));
prepare_plot();
title('Desired before vs after cond Act')


% Plot --- Desired before vs training
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, figureHandlesBefB2, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, figureHandlesBefB2(2), {'b'});
figure(figureHandlesBefB2(1)); 
prepare_plot();
title('Desired before vs training Pos')
figure(figureHandlesBefB2(2)); 
prepare_plot();
title('Desired before vs training Act')


% Plot --- Deisred before vs reproduction
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond1,'actions', 1, figureHandlesBefB3(2), {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond1,'jointPositions', 1, figureHandlesBefB3, {'b'});
figure(figureHandlesBefB3(1));
prepare_plot();
title('Deisred before vs reproduction Pos')
figure(figureHandlesBefB3(2));
prepare_plot();
title('Deisred before vs reproduction Act')

% 
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, figureHandles(2), {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, figureHandles, {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, figureHandles, {'b'});


[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(cond1,'jointVelocities', 1, figureHandles, {'b'});




