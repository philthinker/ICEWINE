%demoTPGMR
%   A demo for TP-GMR
%
%   Haopeng Hu
%   2019.10.24
%   All rights reserved

%% Data Initialization
% Here what we need is the pose of each frame and corresponded data.
% For convenience, the original demo data is assumed to be recorded
% according to the 'world frame', i.e. A = I and b = 0.
% A demo struct contains:
%   A: D x D x F, rotation matrices
%   b: D x F, positions
%   data: N x D, demo data
%   TPData: D x F x N, demo data in each frame
%   Note that in the GMR problem the data always contains a time/decay term

% load('Data\Data02_2d2frame.mat');

%% Learn GMM param.

gmm = TPGMM(3,3,2); % 3 Gaussian, 3d data (t,x1,x2), 2 frame
gmm = gmm.initGMMTimeBased(Demos);
gmm = gmm.learnGMM(Demos);

%% GMR reproduction

query = Demos{1}.data(:,1)';
nData = size(query,2);
qFrame = struct('A',eye(3) ,'b',zeros(3,1));    % Note that there is one more dimension than the data itself reserved for TP-GMR
reprodTPGMM = repmat(struct('data',zeros(2, nData),'Sigma', zeros(2,2,nData),'qFrames', repmat(qFrame,[2,1])),[1,size(Demos,2)]);
for i = 1:size(Demos,2)
    for m = 1:gmm.nFrame
        reprodTPGMM(i).qFrames(m).A = Demos{i}.A(:,:,m);
        reprodTPGMM(i).qFrames(m).b = Demos{i}.b(:,m);
    end
    [reprodTPGMM(i).data,reprodTPGMM(i).Sigma] = gmm.GMR(query,reprodTPGMM(i).qFrames);
end
genProductionTPGMM = reprodTPGMM;
for i = 1:size(genProductionTPGMM,2)
    for m=1:gmm.nFrame
        id = ceil(rand(2,1)*size(Demos,2));
        w = rand(2);
        w = w / sum(w);
        genProductionTPGMM(i).qFrames(m).b = Demos{id(1)}.b(:,m) * w(1) + Demos{id(2)}.b(:,m) * w(2);
        genProductionTPGMM(i).qFrames(m).A = Demos{id(1)}.A(:,:,m) * w(1) + Demos{id(2)}.A(:,:,m) * w(2);
    end
    [genProductionTPGMM(i).data,genProductionTPGMM(i).Sigma] = gmm.GMR(query,genProductionTPGMM(i).qFrames);
end

%% Figure

figure;
xx = round(linspace(1,64,size(Demos,2)));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
limAxes = [-1.2 0.8 -1.1 0.9];
colPegs = [[.9,.5,.9];[.5,.9,.5]];

% Plot Demos
subplot(1,3,1); hold on; box on; title('Demonstrations');
for n=1:size(Demos,2)
	% Plot frames
	for m=1:gmm.nFrame
        plot([Demos{n}.b(2,m) Demos{n}.b(2,m)+Demos{n}.A(2,3,m)], [Demos{n}.b(3,m) Demos{n}.b(3,m)+Demos{n}.A(3,3,m)], '-','linewidth',6,'color',colPegs(m,:));
		plot(Demos{n}.b(2,m), Demos{n}.b(3,m),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	% Plot trajectories
    plot(Demos{n}.data(1,2), Demos{n}.data(1,3),'.','markersize',12,'color',clrmap(n,:));
	plot(Demos{n}.data(:,2), Demos{n}.data(:,3),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

% Plot reproductions
subplot(1,3,2); hold on; box on; title('Reproductions with GMR');
for n=1:size(Demos,2)
	%Plot frames
	for m=1:gmm.nFrame
		plot([Demos{n}.b(2,m) Demos{n}.b(2,m)+Demos{n}.A(2,3,m)], [Demos{n}.b(3,m) Demos{n}.b(3,m)+Demos{n}.A(3,3,m)], '-','linewidth',6,'color',colPegs(m,:));
		plot(Demos{n}.b(2,m), Demos{n}.b(3,m),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:size(reprodTPGMM,2)
	% Plot Gaussians
	plotGMM2SC(reprodTPGMM(n).data(:,1:5:end), reprodTPGMM(n).Sigma(:,:,1:5:end), clrmap(n,:), .05);
end
for n=1:size(reprodTPGMM,2)
	%Plot trajectories
    plot(reprodTPGMM(n).data(1,1), reprodTPGMM(n).data(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(reprodTPGMM(n).data(1,:), reprodTPGMM(n).data(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

% New productions
subplot(1,3,3); hold on; box on; title('New reproductions with GMR');
for n=1:size(genProductionTPGMM,2)
	%Plot frames
	for m=1:gmm.nFrame
		plot([genProductionTPGMM(n).qFrames(m).b(2), genProductionTPGMM(n).qFrames(m).b(2)+genProductionTPGMM(n).qFrames(m).A(2,3)],...
            [genProductionTPGMM(n).qFrames(m).b(3), genProductionTPGMM(n).qFrames(m).b(3)+genProductionTPGMM(n).qFrames(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(genProductionTPGMM(n).qFrames(m).b(2), genProductionTPGMM(n).qFrames(m).b(3),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:size(genProductionTPGMM,2)
	%Plot trajectories
	plot(genProductionTPGMM(n).data(1,1), genProductionTPGMM(n).data(2,1),'.','markersize',12,'color',[.2 .2 .2]);
	plot(genProductionTPGMM(n).data(1,:), genProductionTPGMM(n).data(2,:),'-','linewidth',1.5,'color',[.2 .2 .2]);
end
for n=1:size(genProductionTPGMM,2)
	%Plot Gaussians
	plotGMM2SC(genProductionTPGMM(n).data(:,1:5:end), genProductionTPGMM(n).Sigma(:,:,1:5:end), clrmap(n,:), .05);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

