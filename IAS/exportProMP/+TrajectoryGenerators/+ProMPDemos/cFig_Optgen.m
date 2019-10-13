Common.clearClasses();
% close all;

settings = Common.Settings();

%% Linear Env

settings.setProperty('numTimeSteps', 200);
settings.setProperty('Noise_std', 2.5);
settings.setProperty('dt', 0.005);
settings.setProperty('maxTorque', 5000);

sampler = Sampler.EpisodeWithStepsSampler();
dataManager = sampler.getDataManager();

transitionFunction = Environments.DynamicalSystems.LinearSystem(sampler, 1);
sampler.setTransitionFunction(transitionFunction);

%% Reward function

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

%% Initial state


initialState=[0,0];
initialStateStd = [0.01,0.1];

settings.setProperty('InitialStateDistributionType', 'Gaussian');

settings.setProperty('InitialStateDistributionMinRange', initialState-initialStateStd);
settings.setProperty('InitialStateDistributionMaxRange', initialState+initialStateStd);

initialStateSampler = Sampler.InitialSampler.InitialStateSamplerStandard(sampler);

sampler.setInitialStateSampler(initialStateSampler);


%% Planner

meanTrajectoryPredictor = Planning.MeanTrajectoryForwardPlanner(dataManager, transitionFunction);
linearModelProvider = Planning.FiniteDifferenceLinearizedModelProvider(dataManager, transitionFunction);
rewardProvider = Planning.AnalyticQuadraticRewardModelProvider(dataManager, rewardFunction);

linearizedPlanner = Planning.LinearizedPlanner(dataManager, meanTrajectoryPredictor, linearModelProvider, rewardProvider);

%% Compute Gains

feedbackController = TrajectoryGenerators.TrajectoryTracker.TimeVarLinearController(dataManager, transitionFunction.dimAction, linearizedPlanner);
sampler.setActionPolicy(feedbackController);


%% Sample
sampler.numSamples = 500;
saveplot = 0;

%%
linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);
optData_base = dataManager.getDataObject(0);
sampler.createSamples(optData_base);

Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointPositions', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointVelocities', 1, [], {'r'});

%% Additional via points in POS
rewardFunction.viaPointFactors     = [10^6, 1; 10^5 10^3; 10^6 1]*70;
rewardFunction.viaPoints{3}   = [-0.2 0.0 ];
linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);
optData = dataManager.getDataObject(0);
sampler.createSamples(optData);

[~, ~, fHandP] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointPositions', 1, [], {'r'});
[~, ~, hHandV] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointPositions', 1, fHandP, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointVelocities', 1, hHandV, {'b'});

rewardFunction.viaPoints{3}   = [0.2 0.0 ];
linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);
optData = dataManager.getDataObject(0);
sampler.createSamples(optData);

Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointPositions', 1, fHandP, {'g'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointVelocities', 1, hHandV, {'g'});

%% Additional via points in VEL
rewardFunction.viaPointFactors     = [10^6, 1; 10^5 10^3; 10^1 10^3]*70;
rewardFunction.viaPoints{3}   = [0.0 -1.5 ];
linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);
optData = dataManager.getDataObject(0);
sampler.createSamples(optData);

[~, ~, fHandP] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointPositions', 1, [], {'r'});
[~, ~, hHandV] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointPositions', 1, fHandP, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointVelocities', 1, hHandV, {'b'});

rewardFunction.viaPoints{3}   = [0.0 1.5 ];
linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);
optData = dataManager.getDataObject(0);
sampler.createSamples(optData);

Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointPositions', 1, fHandP, {'g'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointVelocities', 1, hHandV, {'g'});

%% Additional via points in POS + VEL + middle

rewardFunction.viaPointFactors     = [10^6, 1; 10^5 10^3; 10^5 10^3;10^5, 10^4;]*70;
rewardFunction.viaPointTimes = [80 140 200 50];
rewardFunction.viaPoints{3}   = [0.1 2.5 ];
rewardFunction.viaPoints{4}   = [-0.45 -3.0 ];
linearizedPlanner.doPlanning(1, initialState, eye(2) * 10^-6);
optData = dataManager.getDataObject(0);
sampler.createSamples(optData);

[~, ~, fHandP] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointPositions', 1, [], {'r'});
[~, ~, hHandV] = Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData_base,'jointVelocities', 1, [], {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointPositions', 1, fHandP, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(optData,'jointVelocities', 1, hHandV, {'b'});


%% Plots
return

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptCond12',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptCond12Vel',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptCondVel12',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptCondVel12Vel',1);

TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptCondBoth',1);
TrajectoryGenerators.ProMPDemos.prepare_plot('trajDistOptCondBothVel',1);
