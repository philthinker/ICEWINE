Common.clearClasses();
close all;

settings = Common.Settings();

%% Linear Env

settings.setProperty('numTimeSteps', 200);
settings.setProperty('Noise_std', 2.5);
settings.setProperty('dt', 0.005);
settings.setProperty('maxTorque', 5000);

sampler = Sampler.EpisodeWithStepsSampler();
dataManager = sampler.getDataManager();

transitionFunction = Environments.DynamicalSystems.LinearSystem(sampler, 1);

%% Reward function (1st MP)

viaPoint.times       = floor([0.20 0.70 1.0] .*settings.numTimeSteps);
viaPoint.factors     = [10^6, 100; 10^6 100; 10^5 0.1]*70;
viaPoint.points{1}   = [0.1 0.0];
viaPoint.points{2}   = [0.2 0.0 ];
viaPoint.points{3}   = [0.3 0.0 ];

viaPoint.uFactor = 10^-3 / settings.dt;

rewardFunction = RewardFunctions.TimeDependent.ViaPointRewardFunction(dataManager, viaPoint.times,viaPoint.points,viaPoint.factors,viaPoint.uFactor);

returnFuncion = RewardFunctions.ReturnForEpisode.ReturnSummedReward(dataManager);
sampler.setReturnFunction(returnFuncion);

%% Initial state 

initialState=[0,0];
initialStateStd = [0.03,0.3];

settings.setProperty('InitialStateDistributionType', 'Gaussian');

settings.setProperty('InitialStateDistributionMinRange', initialState-initialStateStd);
settings.setProperty('InitialStateDistributionMaxRange', initialState+initialStateStd);

initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);

%% Finalize setup

sampler.setInitialStateSampler(initialStateSampler);
sampler.setTransitionFunction(transitionFunction);
sampler.setRewardFunction(rewardFunction);


meanTrajectoryPredictor = Planning.MeanTrajectoryForwardPlanner(dataManager, transitionFunction);
linearModelProvider = Planning.FiniteDifferenceLinearizedModelProvider(dataManager, transitionFunction);
rewardProvider = Planning.AnalyticQuadraticRewardModelProvider(dataManager, rewardFunction);

linearizedPlanner = Planning.LinearizedPlanner(dataManager, meanTrajectoryPredictor, linearModelProvider, rewardProvider);

%% Compute Gains  (1st MP)

linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);

feedbackController = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, transitionFunction.dimAction, linearizedPlanner);
sampler.setActionPolicy(feedbackController);


%% Sample  (1st MP)

optData1 = dataManager.getDataObject(0);
sampler.numSamples = 100;
sampler.createSamples(optData1);

%% Reward function (2st MP)

rewardFunction.viaPointTimes = floor([0.45 1.00] .*settings.numTimeSteps);
rewardFunction.viaPoints{1} = [-0.1 0.0];
rewardFunction.viaPoints{2} = [ 0.3 0.0];

%% Compute Gains  (2st MP)

linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);

%% Sample  (2st MP)

optData2 = dataManager.getDataObject(0);
sampler.numSamples = 100;
sampler.createSamples(optData2);


%% Reward function (3st MP)

rewardFunction.viaPointTimes    = floor([0.20 0.45 0.70 1.00] .*settings.numTimeSteps);
rewardFunction.viaPointFactors  = [10^6, 100; 10^6 100; 10^6 100; 10^6+10^5 100+0.1]*70;
rewardFunction.viaPoints{1}     = [0.1 0.0];
rewardFunction.viaPoints{2}     = [-0.1 0.0];
rewardFunction.viaPoints{3}     = [0.2 0.0 ];
rewardFunction.viaPoints{4}     = [0.3 0.0];
rewardFunction.uFactor          = 2*10^-3 / settings.dt;

%% Compute Gains  (3st MP)

linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);

%% Sample  (3st MP)

optData3 = dataManager.getDataObject(0);
sampler.numSamples = 500;
sampler.createSamples(optData3);

mean(optData3.getDataEntry('returns'))
std(optData3.getDataEntry('returns'))/sqrt(sampler.numSamples)

%% Plotting

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData1,'jointPositions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData2,'jointPositions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData3,'jointPositions', 1, figureHandles, {'g'});

[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData1,'actions', 1, [], {'b'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData2,'actions', 1, figureHandles, {'r'});
[~, ~, figureHandles] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData3,'actions', 1, figureHandles, {'g'});

 Plotter.PlotterData.plotTrajectories(optData1,'actions', 1);
 Plotter.PlotterData.plotTrajectories(optData2,'actions', 1);
 Plotter.PlotterData.plotTrajectories(optData3,'actions', 1);
 
 %% Save
imData.dataStructure = optData1.dataStructure;
imData.dataStructure.dt = settings.dt;
imData.dataStructure.noise_std = settings.Noise_std;
imData.dataStructure.init_m = initialState;
imData.dataStructure.init_std = initialStateStd;
imData.dataStructure.maxAction = settings.maxTorque;

imData2.dataStructure = optData2.dataStructure;
imData2.dataStructure.dt = settings.dt;
imData2.dataStructure.noise_std = settings.Noise_std;
imData2.dataStructure.init_m = initialState;
imData2.dataStructure.init_std = initialStateStd;
imData2.dataStructure.maxAction = settings.maxTorque;

imData3.dataStructure = optData3.dataStructure;
imData3.dataStructure.dt = settings.dt;
imData3.dataStructure.noise_std = settings.Noise_std;
imData3.dataStructure.init_m = initialState;
imData3.dataStructure.init_std = initialStateStd;
imData3.dataStructure.maxAction = settings.maxTorque;
%  save('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_comb.mat','imData','imData2','imData3')

