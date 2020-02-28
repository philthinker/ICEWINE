%demoProMP0_advan
%   A demo for the advanced usage of ProMPZero
%
%   Haopeng Hu
%   2020.01.27
%   All rights reserved
%{
%% Data init.

% load('Data\LetterS');
M = 9;
N = 200;
D = 2;
Demos = [];
Demos.data = demos{1}.pos; Demos = repmat(Demos,[1,M]);
for i = 1:M
    Demos(i).data = demos{i}.pos;
end
% Note that we DO NOT take the velocity into consideration in this demo.

%% Learn a ProMP

model = ProMPZero(D,20);
model = model.learnLRR(Demos);
%}
%% Advanced operation

[expData,~] = model.reproduct(N);

% Modulation (interpolation and extrapolation)

N_m = 10;
models_m = repmat(model,[1,N_m]);
expData_m = cell(1,N_m);
x_m = [];
x_m.data = expData(:,end);
x_m.t = 1;
x_m = repmat(x_m,[1,N_m]);
for i = 1:N_m
    x_m(i).data = x_m(i).data + [0;0.1*i];
    models_m(i) = model.modulate(x_m(i));
    expData_m{i} = models_m(i).reproduct(N);
end

figure;
for i = 1:M
    plot(Demos(i).data(1,:),Demos(i).data(2,:),'Color',[0.5 0.5 0.5]);
    hold on;
end
plot(expData(1,:),expData(2,:),'Color',[0 0 1]);
for i = 1:N_m
    plot(expData_m{i}(1,:),expData_m{i}(2,:));
    hold on;
end
axis equal;
grid on;

% Modulate ( via-points )

N_v = 10;
models_v = repmat(model,[1,N_v]);
expData_v = cell(1,N_v);
x_v = [];
x_v.data = expData(:,floor(N/2));
x_v.t = 0.5;
x_v = repmat(x_v,[1,N_v]);
for i = 1:N_v
    x_v(i).data = x_v(i).data + [0.2*i;0];
    models_v(i) = model.modulate(x_v(i));
    expData_v{i} = models_v(i).reproduct(N);
end

figure;
for i = 1:M
    plot(Demos(i).data(1,:),Demos(i).data(2,:),'Color',[0.5 0.5 0.5]);
    hold on;
end
plot(expData(1,:),expData(2,:),'Color',[0 0 1]);
for i = 1:N_v
    plot(expData_v{i}(1,:),expData_v{i}(2,:));
    hold on;
end
axis equal;
grid on;
