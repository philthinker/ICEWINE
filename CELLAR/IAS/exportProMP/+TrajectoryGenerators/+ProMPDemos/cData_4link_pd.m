clear variables;
close all;
Common.clearClasses;


%% Config
numSamples = 300;
numPlot = numSamples;
% numPlot = 300;

settings = Common.Settings();
settings.setProperty('PGains', [50, 40, 30, 20]*3);
settings.setProperty('DGains', [3, 3, 3, 3]);
% settings.setProperty('DGains', 0.7*sqrt([50, 40, 30, 20]*3)); %invdyn gains
settings.setProperty('Noise_std', 0.3);
% settings.setProperty('Noise_std', 0);

% Initial dist
init_m = 0;
init_std = 0.01;

% Time 
dt = 0.05;
t_end = 10;
t = dt:dt:t_end;

% Via points
viaPoint_t = [0.5 1.0] * t_end;
viaPoint_pos = [pi / 2 0];
useRand_via = true;
viaRand_std = 0.07;

%% Init

tr_des = zeros(numSamples,length(t), 4);   % Desire
tr = zeros(numSamples,length(t), 4);       % Pos 
trd = zeros(numSamples,length(t), 4);      % Vel
u   = zeros(numSamples,length(t)-1, 4);      % Action
uNoise   = zeros(numSamples,length(t)-1, 4); % Action + action noise

Kp = 950;
Kd = sqrt(Kp);

% init system
tr_1 = randn([numSamples,1, 4]) * init_std;
tr(:,1,:) = tr_1;
tr(:,1,1) = tr(:,1,1) + pi;

%% Run

% Generate desire
for i= 1:numSamples
   viaPoint_pos_i(1,:) = (1 + randn(1) * viaRand_std) * viaPoint_pos;
   viaPoint_pos_i(2,:) = viaPoint_pos_i(1,:) * 1.2 + randn(1) * 0.001;
   viaPoint_pos_i(3,:) = viaPoint_pos_i(2,:) * 1.2 + randn(1) * 0.001;
   viaPoint_pos_i(4,:) = viaPoint_pos_i(3,:) * 1.2 + randn(1) * 0.001;       
   %viaPoint_pos_i(1,1) = viaPoint_pos_i(1,1) + pi; 
   tr_des(i,:,1) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,1),tr(i,1,1),viaPoint_pos_i(1,:) + pi],dt:dt:t_end,'spline'); 
   tr_des(i,:,2) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,2),tr(i,1,2),viaPoint_pos_i(2,:)],dt:dt:t_end,'spline'); 
   tr_des(i,:,3) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,3),tr(i,1,3),viaPoint_pos_i(3,:)],dt:dt:t_end,'spline'); 
   tr_des(i,:,4) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,4),tr(i,1,4),viaPoint_pos_i(4,:)],dt:dt:t_end,'spline'); 

end

tr(:,1, :) = tr(:,1, :) + randn(numSamples,1, 4) * 0.005;

tr_desd = diff(tr_des,1,2)/dt;
tr_desdd = diff(tr_des,2,2)/dt^2;
tr_desddd = diff(tr_des,3,2)/dt^3;

tr_desd(:,end+1, :) = tr_desd(:,end, :)+tr_desdd(:,end, :)*dt;
tr_desdd(:,end+1, :) = tr_desdd(:,end, :)+tr_desddd(:,end, :)*dt;
tr_desdd(:,end+1,: ) = tr_desdd(:,end, :)+tr_desddd(:,end, :)*dt;

% test = tr_des(1,:,1);
% testd = tr_desd(1,:,1);
% testdd= tr_desdd(1,:,1);
% 
% val = zeros(size(test));
% vald = zeros(size(test));
% val(1) = test(1);
% vald(1) = testd(1);
% for i=2:length(test)
%     vald(i) = vald(i-1)+ testdd(i) *dt;
%     val(i) = val(i-1)+ vald(i) *dt;
% end
% 
% figure;hold on;
% plot(test)
% plot(val)

sampler = Sampler.EpisodeWithStepsSampler();
dataManager = sampler.getDataManager();

subDataManager = dataManager.getSubDataManager();
subDataManager.addDataEntry('referencePos', 4);
subDataManager.addDataEntry('referenceVel', 4);
subDataManager.addDataEntry('referenceAcc', 4);

environment = Environments.DynamicalSystems.QuadLink(sampler);
environment.friction = ones(1,4) * 5;
controller = TrajectoryGenerators.TrajectoryTracker.LinearTrajectoryTracker(dataManager, environment.dimAction, environment);

subDataManager.setRestrictToRange('actions', false);
subDataManager.setRange('actions', -[1000, 1000, 1000, 1000], [1000, 1000, 1000, 1000]);
subDataManager.setRestrictToRange('states', false);

% settings.setProperty('useInvDyn', true);
           
environment.initObject();

sampler.setTransitionFunction(environment);
sampler.setActionPolicy(controller);

data = dataManager.getDataObject([numSamples, length(t)]);

sampler.numSamples = numSamples;
% data.setDataEntry('jointPositions', squeeze(tr(:, 1,:)), :, 1);
% data.setDataEntry('jointVelocities', randn(numSamples, 4) * 0.01, :, 1);
data.setDataEntry('jointPositions', squeeze(tr_des(:, 1,:)), :, 1);
data.setDataEntry('jointVelocities', squeeze(tr_desd(:, 1,:)), :, 1);

for i = 1:numSamples
    data.setDataEntry('referencePos', squeeze(tr_des(i, :, :)), i);
    data.setDataEntry('referenceVel', squeeze(tr_desd(i, :, :)), i);
    data.setDataEntry('referenceAcc', squeeze(tr_desdd(i, :, :)), i);
end

dataManager.finalizeDataManager();

sampler.stepSampler.isActiveSampler.numTimeSteps = length(t);
sampler.stepSampler
sampler.createSamples(data);


%% Plotting
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 1, 1, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',1, 1, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 2, 2, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',2, 2, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 3, 3, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',3, 3, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 4, 4, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',4 , 4, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',1 , 5, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',2 , 6, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',3 , 7, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',4 , 8, {'b'});

%% Saving data

dataStructure = data.getDataStructure();
%save('./+TrajectoryGenerators/+test/+ProMPs/im_data_pd_4link.mat','dataStructure')


