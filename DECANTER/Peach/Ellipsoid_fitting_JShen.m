%Ellipsoid_fitting_JShen

% Draw a 3D ellipsoid with GMM

%% Initialization

% Import 'Data\Ellipsoid_fitting_JShen.txt'

addpath('GaussianMixtureModel');

%% Data pre-process

Data = EllipsoidfittingJShen(1:12500,:);
% bias = [1000, 8000, 3000];
bias = [0 0 0];
Data = downsample(Data,2) + bias;
Data = Data';

%% Scatter the raw data

figure;
scatter3(Data(1,:), Data(2,:), Data(3,:));
axis equal;
grid on;
xlabel('x(mm)'); ylabel('y(mm)'); zlabel('z(mm)');
view(3);

%% Fit a GMM

model = GMMOne(1,3);
model = model.initGMMKMeans(Data);
model = model.learnGMM(Data);

%% PlotGMM3

plotGMM3SC(model.Mu,model.Sigma*2.5,[1.0, 0.0, 0.0],0.8);
