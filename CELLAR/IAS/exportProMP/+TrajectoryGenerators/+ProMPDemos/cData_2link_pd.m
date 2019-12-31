clear all;
close all;


%% Config
numSamples = 1000;
numPlot = numSamples;
% numPlot = 300;

settings = Common.Settings();
settings.setProperty('PGains', [30, 30]);
settings.setProperty('DGains', [6, 6]);
settings.setProperty('Noise_std', 0.1);

% Initial dist
init_m = 0;
init_std = 0.1;

% Time 
dt = 0.05;
t_end = 10;
t = dt:dt:t_end;

% Via points
viaPoint_t = [0.40 0.70 1.0] * t_end;
viaPoint_pos = [-0.66, 0.79, 0.0];
useRand_via = true;
viaRand_std = 0.12;

%% Init

tr_des = zeros(numSamples,length(t), 2);   % Desire
tr = zeros(numSamples,length(t), 2);       % Pos 
trd = zeros(numSamples,length(t), 2);      % Vel
u   = zeros(numSamples,length(t)-1, 2);      % Action
uNoise   = zeros(numSamples,length(t)-1, 2); % Action + action noise

Kp = 950;
Kd = sqrt(Kp);

% init system
tr_1 = randn([numSamples,1, 2]) * init_std;
tr(:,1,:) = tr_1;
%% Run

% Generate desire
for i= 1:numSamples
    viaPoint_pos_i = viaPoint_pos;
    if (useRand_via)
        % viaPoint_pos_i = randn(1,length(viaPoint_pos)) * viaRand_std + viaPoint_pos;
%         r = randn(1);
        viaPoint_pos_i(1,:) = (1 + randn(1) * viaRand_std) * viaPoint_pos;
        viaPoint_pos_i(2,:) = - 0.8 * viaPoint_pos_i(1,:) + randn(1, size(viaPoint_pos_i,2)) * 0.01;
        
    end
    tr_des(i,:,1) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,1),tr(i,1,1),viaPoint_pos_i(1,:)],dt:dt:t_end,'spline'); 
    tr_des(i,:,2) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,2),tr(i,1,2),viaPoint_pos_i(2,:)],dt:dt:t_end,'spline'); 
end

tr(:,1, :) = tr(:,1, :) + randn(numSamples,1, 2) * 0.05;

tr_desd = diff(tr_des,1,2)/dt;
tr_desdd = diff(tr_des,2,2)/dt^2;
tr_desddd = diff(tr_des,3,2)/dt^3;

tr_desd(:,end+1, :) = tr_desd(:,end, :)+tr_desdd(:,end, :)*dt;
tr_desdd(:,end+1, :) = tr_desdd(:,end, :)+tr_desddd(:,end, :)*dt;
tr_desdd(:,end+1,: ) = tr_desdd(:,end, :)+tr_desddd(:,end, :)*dt;

sampler = Sampler.EpisodeWithStepsSampler();
dataManager = sampler.getDataManager();

subDataManager = dataManager.getSubDataManager();
subDataManager.addDataEntry('referencePos', 2);
subDataManager.addDataEntry('referenceVel', 2);
subDataManager.addDataEntry('referenceAcc', 2);

environment = Environments.DynamicalSystems.DoubleLink(sampler);
controller = TrajectoryGenerators.TrajectoryTracker.LinearTrajectoryTracker(dataManager, environment.dimAction);

subDataManager.setRestrictToRange('actions', false);
subDataManager.setRange('actions', -[1000, 1000], [1000, 1000]);
           
environment.initObject();

sampler.setTransitionFunction(environment);
sampler.setActionPolicy(controller);

data = dataManager.getDataObject([numSamples, length(t)]);

sampler.numSamples = numSamples;
data.setDataEntry('jointPositions', squeeze(tr(:, 1,:)), :, 1);
data.setDataEntry('jointVelocities', randn(numSamples, 2) * 0.01, :, 1);

for i = 1:numSamples
    data.setDataEntry('referencePos', squeeze(tr_des(i, :, :)), i);
    data.setDataEntry('referenceVel', squeeze(tr_desd(i, :, :)), i);
    data.setDataEntry('referenceAcc', squeeze(tr_desdd(i, :, :)), i);
end


sampler.stepSampler.isActiveSampler.numTimeSteps = length(t);
sampler.stepSampler
sampler.createSamples(data);


%% Plotting
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 1, [], {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 2, [], {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions', 1, [], {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions', 2, [], {'b'});


%% Saving data

dataStructure = data.getDataStructure();
% save('./+TrajectoryGenerators/+test/+ProMPs/im_data_pd_2link.mat','dataStructure')


imData.dataStructure = data.dataStructure;
imData.dataStructure.dt = dt;
imData.dataStructure.noise_std = settings.Noise_std;
imData.dataStructure.init_m = [init_m, init_m];
imData.dataStructure.init_std = [init_std, init_std];
imData.dataStructure.maxAction = 10*max(abs(u(:)));
%  save('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_opt.mat','imData')

