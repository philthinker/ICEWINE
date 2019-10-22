Common.clearClasses();
close all;

settings = Common.Settings();

%% Linear Env

settings.setProperty('numTimeSteps', 200);
settings.setProperty('Noise_std', 3.5);
settings.setProperty('dt', 0.005);
settings.setProperty('maxTorque', 5000);

sampler = Sampler.EpisodeWithStepsSampler();
dataManager = sampler.getDataManager();

transitionFunction = Environments.DynamicalSystems.LinearSystem(sampler, 1);

%% Reward function

viaPoint.times       = floor([0.40 0.70 1.0] .*settings.numTimeSteps);
viaPoint.factors     = [10^5, 1; 10^6 1; 10^5 10^2]*70;
viaPoint.points{1}   = [-0.7 0.0];
viaPoint.points{2}   = [0.8 0.0 ];
viaPoint.points{3}   = [0.0 0.0 ];

viaPoint.uFactor = 10^-3 / settings.dt;

rewardFunction = RewardFunctions.TimeDependent.ViaPointRewardFunction(dataManager, viaPoint.times,viaPoint.points,viaPoint.factors,viaPoint.uFactor);

%% Initial state 

if ( 0 )
    initialState=[0,0];
    initialStateStd = [0.01,0.1];
    
    settings.setProperty('InitialStateDistributionType', 'Gaussian');
    
    settings.setProperty('InitialStateDistributionMinRange', initialState-initialStateStd);
    settings.setProperty('InitialStateDistributionMaxRange', initialState+initialStateStd);
    
    initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);
else
    %     load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat')
    load('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_optRecInit.mat')    
    nSamples = imData.dataStructure.numElements;
    trLength = length(imData.dataStructure.steps(1).states);
    trData = dataManager.getDataObject([nSamples trLength]);
    trData.copyValuesFromDataStructure(imData.dataStructure);    
    settings.setProperty('useLastState', 'true');
    initialStateSampler = Sampler.InitialSampler.InitialStateSamplerFromTrainingData(sampler,trData);
    initialState = mean(trData.getDataEntry('states',:,1));
    clear imData; clear trData;
    sampler.setInitialStateSampler(initialStateSampler);    
end


%% Finalize setup

sampler.setInitialStateSampler(initialStateSampler);
sampler.setTransitionFunction(transitionFunction);
sampler.setRewardFunction(rewardFunction);


meanTrajectoryPredictor = Planning.MeanTrajectoryForwardPlanner(dataManager, transitionFunction);
linearModelProvider = Planning.FiniteDifferenceLinearizedModelProvider(dataManager, transitionFunction);
rewardProvider = Planning.AnalyticQuadraticRewardModelProvider(dataManager, rewardFunction);

linearizedPlanner = Planning.LinearizedPlanner(dataManager, meanTrajectoryPredictor, linearModelProvider, rewardProvider);

%% Compute Gains

linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);

feedbackController = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, transitionFunction.dimAction, linearizedPlanner);
sampler.setActionPolicy(feedbackController);


%% Sample

optData = dataManager.getDataObject(0);
sampler.numSamples = 100;
sampler.createSamples(optData);

%% Plotting

[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointPositions', 1, [], {'r'});
[meanValues, stdValues, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'actions', 1, [], {'r'});
 Plotter.PlotterData.plotTrajectories(optData,'actions', 1);
 
 %% Save
imData.dataStructure = optData.dataStructure;
imData.dataStructure.dt = settings.dt;
imData.dataStructure.noise_std = settings.Noise_std;
imData.dataStructure.init_m = initialState;
% imData.dataStructure.init_std = initialStateStd;
imData.dataStructure.maxAction = settings.maxTorque;
%  save('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat','imData')
%  save('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt2K.mat','imData')
save('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_optRecInit.mat','imData')
