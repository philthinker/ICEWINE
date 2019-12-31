clear variables;
close all;
Common.clearClasses;

settings = Common.Settings();

%% Load demonstrations
% load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat')
% % load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_pd.mat')
% 
% dt           = imData.dataStructure.dt;
% numTimeSteps = imData.dataStructure.steps(1).numElements;
% Noise_std    = imData.dataStructure.noise_std;
% maxAction    = imData.dataStructure.maxAction;
% initState    = imData.dataStructure.init_m;
% initStateStd = imData.dataStructure.init_std;

%% Env porperties (same as demonstrated system)

settings.setProperty('dt', 0.005);
settings.setProperty('numTimeSteps', 200);
settings.setProperty('numBasis', 50);
settings.setProperty('widthFactorBasis', 1.5);
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


dataManager.addDataEntry('steps.jointStateAction', dimObsState+dimCtls);

sampler.setParameterPolicy(gainGenerator,'updateModel');

sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisDD');
sampler.setParameterPolicy(trajectoryGenerator.basisGenerator,'generateBasisD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseDD');
sampler.setParameterPolicy(trajectoryGenerator.phaseGenerator,'generatePhaseD');


dataManager.finalizeDataManager();

%%% From file
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_1k.mat'  )
% load('+TrajectoryGenerators/+test/+ProMPs/im_data_opt_smooth_1k.mat'  )
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
%    figure(100);plot(imlearn.q{i}(:,1));
    if (useNoise)
        imlearn.uNoise{i}(end+1,:) = imlearn.uNoise{i}(end,:);
        %figure(101);plot(imlearn.uNoise{i});
            trData.setDataEntry('jointStateAction', ...
                         [imlearn.q{i}(:,1:2:end) , imlearn.uNoise{i}], i);
            trData.setDataEntry('actions', imlearn.uNoise{i}, i);
    else
        d = imlearn.u{i}(end,:) - imlearn.u{i}(end-1,:);
        imlearn.u{i}(end+1,:) = imlearn.u{i}(end,:)+d;
        %figure(101);plot(imlearn.u{i});
            trData.setDataEntry('jointStateAction', ...
                         [imlearn.q{i}(:,1:2:end) , imlearn.u{i}], i);
        trData.setDataEntry('actions', imlearn.u{i}, i);
    end
                     
    trData.setDataEntry('jointPositions', imlearn.q{i}(:,1:2:end), i);  %for comp plotting
    trData.setDataEntry('jointVelocities', imlearn.q{i}(:,2:2:end), i);
end

sampler.numSamples = 300;
sampler.setParallelSampling(true);

imitationLearnerDistribution.updateModel(trData);



tic
sampler.createSamples(sampleData);
toc


% Plot --- Desired vs training
fig = trajectoryGenerator.plotStateDistribution(trData);
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointPositions', 1, fig, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'actions', 1, fig(2), {'r'});
figure(fig(1));title('Desired vs training Pos')
figure(fig(2));title('Desired vs training Act')





% trajectoryGenerator.plotStateDistribution(trData)

% Plot --- Desired vs reprod
fig = trajectoryGenerator.plotStateDistribution(sampleData);
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointPositions', 1, fig, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'actions', 1, fig(2), {'r'});
figure(fig(1));title('Desired vs reprod Pos')
figure(fig(2));title('Desired vs reprod Act')

fig = trajectoryGenerator.plotStateDistribution(sampleData,1);
Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, fig, {'r'});
figure(fig(1));title('Desired vs reprod Vel')



% 
% % 
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(trData,'jointVelocities', 1, [], {'r'});
% [meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(sampleData,'jointVelocities', 1, figureHandles, {'b'});
