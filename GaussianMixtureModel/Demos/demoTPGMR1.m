%demoTPGMR1 A demo for class @TPGMMOne
%
% Haopeng Hu
% 2020.03.06
% All rights reserved
%

%% Data init.
%   TPDemo struct:
%   |   data: D x N, demo data in world frame
%   |   A: D x D x F, orientation matrices
%   |   b: D x F, position vectors
%   |   TPData: D x F x N, demo data in each frame
%   Note that for GMR usage, the time/decaying term is supposed to be the
%   first row of data.

% load('Data\Data03_2d2frame.mat');

%% Init. TP-GMM

tpgmm = TPGMMOne(3,3,2);    % K, D, F
tpgmm = tpgmm.initGMMTimeBased(Demos);
tpgmm = tpgmm.learnGMM(Demos);

%% Reproduction

query = Demos(1).data(1,:);
nData = size(query,2);  % N
M = size(Demos,2);
% Reproduction
qFrame = struct('A',eye(3) ,'b',zeros(3,1));
reprodTPGMM = repmat(struct('data',zeros(2, nData),'Sigma', zeros(2,2,nData),'qFrames', repmat(qFrame,[2,1])),[1,M]);
for i = 1:M
    for f = 1:tpgmm.nFrame
        reprodTPGMM(i).qFrames(f).A = Demos(i).A(:,:,f);
        reprodTPGMM(i).qFrames(f).b = Demos(i).b(:,f);
    end
    [reprodTPGMM(i).data,reprodTPGMM(i).Sigma] = tpgmm.GMR(query,reprodTPGMM(i).qFrames);
end
% Generalization (Interpolation)
genProductionTPGMM = reprodTPGMM;
for i = 1:size(genProductionTPGMM,2)
    for f=1:tpgmm.nFrame
        id = ceil(rand(2,1)*size(Demos,2));
        w = rand(2);
        w = w / sum(w);
        genProductionTPGMM(i).qFrames(f).b = Demos(id(1)).b(:,f) * w(1) + Demos(id(2)).b(:,f) * w(2);
        genProductionTPGMM(i).qFrames(f).A = Demos(id(1)).A(:,:,f) * w(1) + Demos(id(2)).A(:,:,f) * w(2);
    end
    [genProductionTPGMM(i).data,genProductionTPGMM(i).Sigma] = tpgmm.GMR(query,genProductionTPGMM(i).qFrames);
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
for n=1:M
	% Plot frames
	for f=1:tpgmm.nFrame
        plot([Demos(n).b(2,f) Demos(n).b(2,f)+Demos(n).A(2,3,f)], [Demos(n).b(3,f) Demos(n).b(3,f)+Demos(n).A(3,3,f)], '-','linewidth',6,'color',colPegs(f,:));
		plot(Demos(n).b(2,f), Demos(n).b(3,f),'.','markersize',30,'color',colPegs(f,:)-[.05,.05,.05]);
	end
	% Plot trajectories
    plot(Demos(n).data(2,1), Demos(n).data(3,1),'.','markersize',12,'color',clrmap(n,:));
	plot(Demos(n).data(2,:), Demos(n).data(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

% Plot reproductions
subplot(1,3,2); hold on; box on; title('Reproductions with GMR');
for n=1:M
	%Plot frames
	for f=1:tpgmm.nFrame
		plot([Demos(n).b(2,f) Demos(n).b(2,f)+Demos(n).A(2,3,f)], [Demos(n).b(3,f) Demos(n).b(3,f)+Demos(n).A(3,3,f)], '-','linewidth',6,'color',colPegs(f,:));
		plot(Demos(n).b(2,f), Demos(n).b(3,f),'.','markersize',30,'color',colPegs(f,:)-[.05,.05,.05]);
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
	for f=1:tpgmm.nFrame
		plot([genProductionTPGMM(n).qFrames(f).b(2), genProductionTPGMM(n).qFrames(f).b(2)+genProductionTPGMM(n).qFrames(f).A(2,3)],...
            [genProductionTPGMM(n).qFrames(f).b(3), genProductionTPGMM(n).qFrames(f).b(3)+genProductionTPGMM(n).qFrames(f).A(3,3)], '-','linewidth',6,'color',colPegs(f,:));
		plot(genProductionTPGMM(n).qFrames(f).b(2), genProductionTPGMM(n).qFrames(f).b(3),'.','markersize',30,'color',colPegs(f,:)-[.05,.05,.05]);
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

