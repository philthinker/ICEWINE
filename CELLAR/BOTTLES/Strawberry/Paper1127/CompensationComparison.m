%CompensationComparison

% Raw data: 'Paper2011\Data\paper2011.mat'
% Output data: 'Paper1127\Data\paper2011-gpr.mat'

%% Raw data show
%{
for i = 1:7
    subplot(7,1,i);
    plot(segTestData(2).time, segTestData(2).pose(:,i),'Color',[0, 0, 1]);
    grid on;
    axis([-Inf,Inf,-Inf,Inf]);
end
%}

%% Comparison %%

% Method 1: Spline + Slerp
% Method 2: GRP

%% Set cut point

cut = (95:133);

%% Interpolation

% Data init.
N = 200;
segTestData(3) = segTestData(2);    % qw qx qy qz x y z
segTestData(3).text = 'interp_pose';
segTestData(3).dataGPCompen = [];
segTestData(3).dataCompen.p = zeros(N,3);
segTestData(3).dataCompen.q = [];
% Spline
t = linspace(segTestData(3).time(1), segTestData(3).time(end),200)';
for i = 1:3
    segTestData(3).dataCompen.p(:,i) = spline(segTestData(3).timeGap, segTestData(3).dataGap(:,4+i), t);
end
% Slerp
Ncut = length(cut);
segTestData(3).dataCompen.q = zeros(Ncut,4);
for i = 1:Ncut
    segTestData(3).dataCompen.q(i,:) = quatinterp(...
        quatnormalize(segTestData(3).pose(cut(1),1:4)), ...
        quatnormalize(segTestData(3).pose(cut(end),1:4)),...
        i/Ncut);
end
% Show
figure;
ylabels = {'qw','qx','qy','qz','x','y','z'};
% % Plot raw data
for i = 1:7
    subplot(7,1,i);
    scatter(segTestData(3).time, segTestData(3).pose(:,i),36,'g','.');
    hold on;
    scatter(segTestData(3).timeGap, segTestData(3).dataGap(:,i),36,'b','.');
    grid on; axis([-Inf,Inf,-Inf,Inf]);
    ylabel(ylabels{i});
    if i == 7
        xlabel('t');
    end
end
% % Plot interpolated data
for i = 1:4
    subplot(7,1,i);
    scatter(segTestData(3).timeGap, segTestData(3).dataGap(:,i),36,'r','.');
    for j = 1:Ncut
        scatter(segTestData(3).time(cut(j)), segTestData(3).dataCompen.q(j,i),36,'r','.');
    end
end
for i = 1:3
    subplot(7,1,i+4);
    scatter(t,segTestData(3).dataCompen.p(:,i),36,'r','.');
end

% Compute SEA


%% GPR estimation

% GRP


% Compute SEA
% segTestData(2).SEA = zeros(1,7);
% for i = 1:7
%     segTestData(2).SEA(i) = compSEA(t
% end
