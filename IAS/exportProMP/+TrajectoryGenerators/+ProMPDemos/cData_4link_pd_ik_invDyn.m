clear variables;
close all;
Common.clearClasses;



% Initial dist
init_m = 0;
init_std = 0.4;

% Time 
dt = 0.05;
t_end = 10;
t = dt:dt:t_end;

% Via points
viaPoint_t = [0.3 0.8 1.0] * t_end;
viaPoint_pos = [pi/2 pi/2 0];
useRand_via = true;
viaRand_std = 0.01;

numSamples = 60;
numPlot = numSamples;

%% Init

tr_des = zeros(numSamples,length(t), 2);   % Desire
tr = zeros(numSamples,length(t), 2);       % Pos 
trd = zeros(numSamples,length(t), 2);      % Vel
u   = zeros(numSamples,length(t)-1, 2);      % Action
uNoise   = zeros(numSamples,length(t)-1, 2); % Action + action noise


% init system
tr_1 = randn([numSamples,1, 2]) * init_std;
tr(:,1,:) = tr_1;
tr(:,1,1) = tr(:,1,1) + pi;

%% Run

% Generate desire
for i= 1:numSamples
   viaPoint_pos_i(1,:) = (1 + randn(1) * viaRand_std) * viaPoint_pos;   
   viaPoint_pos_i(2,:) = viaPoint_pos_i(1,:) * 1.2 + randn(1) * 0.001;   
   viaPoint_pos_i(:,end) = randn(1,2) .* 0.4 + viaPoint_pos_i(:,2)';
   tr_des(i,:,1) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,1),tr(i,1,1),viaPoint_pos_i(1,:) + pi],dt:dt:t_end,'spline'); 
   tr_des(i,:,2) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1,2),tr(i,1,2),viaPoint_pos_i(2,:)],dt:dt:t_end,'spline'); 
end

% tr(:,1, :) = tr(:,1, :) + randn(numSamples,1, 2) * 0.005;
tr_des(:,:,2) = tr_des(:,:,2)*0.7+0.8;
tr_des(:,:,1) = repmat(linspace(1,3,size(tr_des,2)),numSamples,1);


tr_desd = diff(tr_des,1,2)/dt;
tr_desdd = diff(tr_des,2,2)/dt^2;
tr_desddd = diff(tr_des,3,2)/dt^3;

tr_desd(:,end+1, :) = tr_desd(:,end, :)+tr_desdd(:,end, :)*dt;
tr_desdd(:,end+1, :) = tr_desdd(:,end, :)+tr_desddd(:,end, :)*dt;
tr_desdd(:,end+1,: ) = tr_desdd(:,end, :)+tr_desddd(:,end, :)*dt;

%% Plot init

figure;plot(tr_des(:,:,1)')
figure;plot(tr_des(:,:,2)')
figure;plot(tr_des(:,:,1)',tr_des(:,:,2)')
% figure;plot(linspace(1,3,size(tr_des,2)),tr_des(:,:,2)*0.7+0.8)
% keyboard
%% Config

settings = Common.Settings();
settings.setProperty('PGains', [50, 40, 30, 20]*3);
settings.setProperty('DGains', sqrt(0.7*[50, 40, 30, 20]*3));
% settings.setProperty('DGains', [3, 3, 3, 3]);
settings.setProperty('Noise_std', 0.03);

settings.setProperty('useInvDyn', true);



sampler = Sampler.EpisodeWithStepsSampler();
dataManager = sampler.getDataManager();

subDataManager = dataManager.getSubDataManager();
subDataManager.addDataEntry('referencePos', 4);
subDataManager.addDataEntry('referenceVel', 4);
subDataManager.addDataEntry('referenceAcc', 4);

subDataManager.addDataEntry('cartPos', 2);

environment = Environments.DynamicalSystems.QuadLink(sampler);
environment.friction = ones(1,4) * 5;
controller = TrajectoryGenerators.TrajectoryTracker.LinearTrajectoryTracker(dataManager, environment.dimAction,environment);
% controller = TrajectoryGenerators.TrajectoryTracker.InvKinTracker(dataManager, environment);

subDataManager.setRestrictToRange('actions', false);
subDataManager.setRange('actions', -[1000, 1000, 1000, 1000], [1000, 1000, 1000, 1000]);
subDataManager.setRestrictToRange('states', false);
           
environment.initObject();

sampler.setTransitionFunction(environment);
sampler.setActionPolicy(controller);

data = dataManager.getDataObject([numSamples, length(t)]);

sampler.numSamples = numSamples;
initPos = zeros(numSamples,size(tr_des,2),4);
tic
for i = 1:numSamples
    for l = 1:size(tr_des,2)
        for j = 1:6*1e1
            err = squeeze(tr_des(i,l,:)) - environment.getForwardKinematics(squeeze(initPos(i,l,:))')';
            if ( norm(err) < 1e-1 )
                break;
            end
            J = environment.getJacobian(squeeze(initPos(i,l,:))');
            %             vel = 1e-2 * J' * err;
            Jps = J'/(J*J'+1e-6*eye(2));
            vel = 1e-2 * Jps * err + 1e-3*(eye(4)- Jps*J) *  -squeeze(initPos(i,l,:)) ;
            initPos(i,l,:) = squeeze(initPos(i,l,:))' + vel';
        end
        if ( j == 1e3 )
            keyboard
        end
    end
end
toc
figure;plot(squeeze(initPos(:,:,1))')

initPosd = diff(initPos,1,2)/dt;
initPosdd = diff(initPos,2,2)/dt^2;
initPosddd = diff(initPos,3,2)/dt^3;

initPosd(:,end+1, :) = initPosd(:,end, :)+initPosdd(:,end, :)*dt;
initPosdd(:,end+1, :) = initPosdd(:,end, :)+initPosddd(:,end, :)*dt;
initPosdd(:,end+1,: ) = initPosdd(:,end, :)+initPosddd(:,end, :)*dt;

sampler.numSamples = numSamples;
% data.setDataEntry('jointPositions', squeeze(tr(:, 1,:)), :, 1);
% data.setDataEntry('jointVelocities', randn(numSamples, 4) * 0.01, :, 1);
data.setDataEntry('jointPositions', squeeze(initPos(:, 1,:)), :, 1);
data.setDataEntry('jointVelocities', squeeze(initPosd(:, 1,:)), :, 1);
% data.setDataEntry('jointVelocities', randn(numSamples, 4) * 0.01, :, 1);


for i = 1:numSamples
    data.setDataEntry('referencePos', squeeze(initPos(i, :, :)), i);
    data.setDataEntry('referenceVel', squeeze(initPosd(i, :, :)), i);
    data.setDataEntry('referenceAcc', squeeze(initPosdd(i, :, :)), i);
end

dataManager.finalizeDataManager();

sampler.stepSampler.isActiveSampler.numTimeSteps = length(t);
sampler.stepSampler
sampler.createSamples(data);


%%
% data.setDataEntry('jointPositions', initPos, :, 1);
% % data.setDataEntry('jointPositions', squeeze(tr(:, 1,:)), :, 1);
% data.setDataEntry('jointVelocities', randn(numSamples, 4) * 0.01, :, 1);
% 
% for i = 1:numSamples
%     data.setDataEntry('referencePos', squeeze(tr_des(i, :, :)), i);
%     data.setDataEntry('referenceVel', squeeze(tr_desd(i, :, :)), i);
%     data.setDataEntry('referenceAcc', squeeze(tr_desdd(i, :, :)), i);
% end
% 
% dataManager.finalizeDataManager();
% 
% sampler.stepSampler.isActiveSampler.numTimeSteps = length(t);
% sampler.stepSampler
% sampler.createSamples(data);


%% Plotting
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 1, 31, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',1, 31, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 2, 32, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 3, 33, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointPositions', 4, 34, {'r'});


jointPos = data.getDataEntry3D('jointPositions');
for i = 1:numSamples
    cartPos = environment.getForwardKinematics(squeeze(jointPos(i,:,:)));
    data.setDataEntry('cartPos', cartPos, i);
end

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'cartPos', 1, 15, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',1, 15, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'cartPos', 2, 16, {'r'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'referencePos',2, 16, {'b'});

Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',1, 20, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',2, 21, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',3, 22, {'b'});
Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'actions',4, 23, {'b'});

% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefPos',1, figure, {'b'});
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefPos',2, figure, {'b'});
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefPos',3, figure, {'b'});
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefPos',4, figure, {'b'});
% 
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefVel',1, figure, {'b'});
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefVel',2, figure, {'b'});
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefVel',3, figure, {'b'});
% Plotter.PlotterData.plotTrajectoriesMeanAndStd(data,'jointRefVel',4, figure, {'b'});
% 
% Plotter.PlotterData.plotTrajectories(data, 'jointRefPos', 3, figure, 1)
% jointQ = data.getDataEntry('jointRefPos',1);
% figure;plot(jointQ)
% figure;plot(diff(jointQ))

%% Saving data

dataStructure = data.getDataStructure();
% save('./+TrajectoryGenerators/+test/+ProMPs/im_data_pd_4link_inv_ik2.mat','dataStructure')


