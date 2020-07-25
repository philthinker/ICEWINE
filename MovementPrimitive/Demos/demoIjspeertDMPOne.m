%demoIjspeertDMPOne
%   A demo for class @IjspeertDMPOne
%   Haopeng Hu
%   2020.02.25
%   All rights reserved

% load('Data\SimpleMinJerk.mat');

%% Init. a DMP

dmp = IjspeertDMPOne(40);
dmp.plotBasis();

%% Run an empty DMP

[Y,x] = dmp.run(0,1,1000);
dmp.plotDMP(x,Y);

%% Run an empty DMP iteratively

y0 = 1; g = 0; dt = 0.001; N = 1000;
x = zeros(N,1); x(1) = 1; y = y0*ones(N,1); dy = zeros(N,1); ddy = zeros(N,1);
t = 1;
while abs(y(t)-g)>1e-3 && t < N
    t = t + 1;
    [x(t),ddy(t),dy(t),y(t)] = dmp.runItera(x(t-1),y(t-1),dy(t-1),y0,g,dt);
end
N = t;
y = y(1:N); dy = dy(1:N); ddy = ddy(1:N);
dmp.plotDMP(x,[y,dy,ddy]);

%% Learn a minimal jerk trajectory

dmp = dmp.learnLWR(T);
[Y,~,fx] = dmp.run(T(1,1),T(end,1),size(T,1));
dmp.plotBasis(fx);
dmp.plotComparison(Y,T);

%% Generalize to other goal position

N = 1000;
[Y1] = dmp.run(0,1,N);
[Y2] = dmp.run(0,2,N);
[Y3,x] = dmp.run(0,3,N);
t = linspace(0,1,N)';
figure;
titles = { 'y', 'dy', 'ddy' };
for i = 1:3
    subplot(3,1,i);
    plot(t,[Y1(:,i),Y2(:,i),Y3(:,i)]); title(titles{i});
    aa = axis; axis([min(t),max(t),aa(3:4)]);
end

%% Generalize to other goal position iteratively

% Init.
y0 = 0; g1 = 1; g2 = 2; g3 = 3; dt = 0.001; N = 1000;
x1 = zeros(N,1); x1(1) = 1; x2 = x1; x3 = x2;
y1 = y0*ones(N,1); dy1 = zeros(N,1); ddy1 = zeros(N,1);
y2 = y0*ones(N,1); dy2 = zeros(N,1); ddy2 = zeros(N,1);
y3 = y0*ones(N,1); dy3 = zeros(N,1); ddy3 = zeros(N,1);
% Update
t = 1; flag = true; flags = true(3,1);
while flag && t < N
    t = t + 1;
    if abs(y1(t-1) - g1) > 1e-3 && flags(1)
        [x1(t),ddy1(t),dy1(t),y1(t)] = dmp.runItera(x1(t-1),y1(t-1),dy1(t-1),y0,g1,dt);
    elseif flags(1)
        N1 = t-1;
        flags(1) = false;
    end
    if abs(y2(t-1) - g2) > 1e-3 && flags(2)
        [x2(t),ddy2(t),dy2(t),y2(t)] = dmp.runItera(x2(t-1),y2(t-1),dy2(t-1),y0,g2,dt);
    elseif flags(2)
        N2 = t-1;
        flags(2) = false;
    end
    if abs(y3(t-1) - g3) > 1e-3 && flags(3)
        [x3(t),ddy3(t),dy3(t),y3(t)] = dmp.runItera(x3(t-1),y3(t-1),dy3(t-1),y0,g3,dt);
    elseif flags(3)
        N3 = t-1;
        flags(3) = false;
    end
    flag = flags(1) || flags(2) || flags(3);
end
x1 = x1(1:N1); x2 = x2(1:N2); x3 = x3(1:N3);
y1 = y1(1:N1); dy1 = dy1(1:N1); ddy1 = ddy1(1:N1);
y2 = y2(1:N2); dy2 = dy2(1:N2); ddy2 = ddy2(1:N2);
y3 = y3(1:N3); dy3 = dy3(1:N3); ddy3 = ddy3(1:N3);
dmp.plotDMP(x1,[y1,dy1,ddy1]);
dmp.plotDMP(x2,[y2,dy2,ddy2]);
dmp.plotDMP(x3,[y3,dy3,ddy3]);
