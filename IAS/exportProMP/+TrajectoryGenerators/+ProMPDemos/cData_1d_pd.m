%% Ad-hoc script for data creation simulating a noisy 1d linear system.

close all;

%% Config
numSamples = 1000;
numPlot = numSamples;

% Noise 
noise_std = 0.5;

% Initial dist
init_m = 0;
init_std = 0.1;

% Time 
dt = 0.005;
t_end = 1;
t = dt:dt:t_end;

% Via points
viaPoint_t = [0.40 0.70 1.0];
viaPoint_pos = [-0.66, 0.79, 0.0];
useRand_via = true;
viaRand_std = 0.12;

%% Init

tr_des = zeros(numSamples,length(t));   % Desire
tr = zeros(numSamples,length(t));       % Pos 
trd = zeros(numSamples,length(t));      % Vel
u   = zeros(numSamples,length(t)-1);      % Action
uNoise   = zeros(numSamples,length(t)-1); % Action + action noise

Kp = 950;
Kd = sqrt(Kp);

% init system
tr(:,1) = randn(numSamples,1) * init_std;

%% Run

% Generate desire
for i= 1:numSamples
    viaPoint_pos_i = viaPoint_pos;
    if (useRand_via)
        % viaPoint_pos_i = randn(1,length(viaPoint_pos)) * viaRand_std + viaPoint_pos;
%         r = randn(1);
        viaPoint_pos_i = (1 + randn(1) * viaRand_std) * viaPoint_pos;
    end
    tr_des(i,:) = interp1([dt, 2*dt,viaPoint_t],[tr(i,1),tr(i,1),viaPoint_pos_i],dt:dt:1,'spline'); 
end

tr(:,1) = tr(:,1) + randn(numSamples,1) *  noise_std / sqrt(dt) * dt^2;

tr_desd = diff(tr_des,1,2)/dt;
tr_desdd = diff(tr_des,2,2)/dt^2;
tr_desddd = diff(tr_des,3,2)/dt^3;

tr_desd(:,end+1) = tr_desd(:,end)+tr_desdd(:,end)*dt;
tr_desdd(:,end+1) = tr_desdd(:,end)+tr_desddd(:,end)*dt;
tr_desdd(:,end+1) = tr_desdd(:,end)+tr_desddd(:,end)*dt;

% Sim system --- sympletic euler
for i= 1:(length(t)-1)
    
    if(i==1)
        u(:,i) = [(tr_des(:,i)-tr(:,i)), tr_desd(:,i)-trd(:,i)] * [Kp, Kd]' +0*0.1* tr_desdd(:,i);
    else
        u(:,i) = [(tr_des(:,i)-tr(:,i)), tr_desd(:,i)-trd(:,i)] * [Kp, Kd]' +0* tr_desdd(:,i);
    end
    uNoise(:,i) = u(:,i) + randn(numSamples,1) * noise_std / sqrt(dt);
       
    trd(:,i+1) = trd(:,i) + uNoise(:,i) * dt;
    tr(:,i+1) = tr(:,i) + trd(:,i+1) * dt;
end


%% Plotting

%Plot Des Pos
figure;hold all;
title('Des Pos')
for i = 1:numPlot
    plot( t, tr_des(i,:) )
end

%Plot Real
figure;hold all;
title('Pos')
for i = 1:numPlot
    plot( t, tr(i,:) )
end

%Plot Real
figure;hold all;
title('Act')
for i = 1:numPlot
    plot( t(1:end-1), u(i,:) )
end

%% Saving data

imData = struct();
imData.dataStructure = struct();
imData.dataStructure.numElements = numSamples;
imData.dataStructure.dt = dt;
imData.dataStructure.noise_std = noise_std;
imData.dataStructure.init_m = [init_m, init_m];
imData.dataStructure.init_std = [init_std, init_std];
imData.dataStructure.maxAction = 10*max(abs(u(:)));
for i=1:numSamples
    imData.dataStructure.steps(i,1).numElements = length(tr(i,:));
    imData.dataStructure.steps(i,1).states = [ tr(i,:); trd(i,:) ]';
    imData.dataStructure.steps(i,1).actions = u(i,:)';    
end

% save('+TrajectoryGenerators/+ProMPDemos/data/im_data_1d_pd.mat','imData')

