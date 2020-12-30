function [] = plotHMM2SC(Trans, StatePrior)
%plotHMM2SC Plot the transition graph of an HMM
%   Trans: K x K matrix, transition probabilisitc matrix
%   StatePrior: 1 x K, the state prior

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

nbStates = size(Trans,1);
valAlpha = 1;
graphRadius = 1;
nodeRadius = .2;
nodePts = 40;
%nodeAngle = linspace(pi,2*pi+pi,nbStates+1);
nodeAngle = linspace(pi/2,2*pi+pi/2,nbStates+1);
for i=1:nbStates
	nodePos(:,i) = [cos(nodeAngle(i)); sin(nodeAngle(i))] * graphRadius;
end
clrmap = lines(nbStates);
ff = .8 / max(max(Trans-diag(diag(Trans))));

for i=1:nbStates
	%Plot StatePrior 
	posTmp = [cos(nodeAngle(i)); sin(nodeAngle(i))] * graphRadius + ...
		[cos(nodeAngle(i)+pi/3); sin(nodeAngle(i)+pi/3)] * nodeRadius*2;
	dirTmp = nodePos(:,i) - posTmp;
	dirTmp = (norm(dirTmp)-nodeRadius) * dirTmp/norm(dirTmp);
	plot2DArrowSC(posTmp, dirTmp, max([.8 .8 .8]-StatePrior(i)*.8,0)); 
	
	%Plot self-transitions
	d = nodeRadius*1.2;
	posTmp = [cos(nodeAngle(i)); sin(nodeAngle(i))] * (graphRadius+d);
	R = nodeRadius;
	r = nodeRadius*0.5; 
	aTmp = asin((4*d^2*R^2-(d^2-r^2+R^2)^2)^.5/(2*d*r));
	a = linspace(nodeAngle(i)+pi-aTmp, nodeAngle(i)-pi+aTmp, nodePts);
	meshTmp = [cos(a); sin(a)] * r + repmat(posTmp,1,nodePts);
	plot(meshTmp(1,:), meshTmp(2,:), 'linewidth', 2, 'color', max([.8 .8 .8]-Trans(i,i)*ff,0)); 
	plot2DArrowSC(meshTmp(:,end-10), meshTmp(:,end)-meshTmp(:,end-10), max([.8 .8 .8]-Trans(i,i)*ff,0)); 

	for j=i+1:nbStates
		%Plot Trans
		dirTmp = nodePos(:,j)-nodePos(:,i);
		dirTmp = (norm(dirTmp)-2*nodeRadius) * dirTmp/norm(dirTmp);
		offTmp = [dirTmp(2); -dirTmp(1)] / norm(dirTmp);
		posTmp = nodePos(:,i) + nodeRadius * dirTmp/norm(dirTmp) + offTmp*0.05;
		plot2DArrowSC(posTmp, dirTmp, max([.8 .8 .8]-Trans(i,j)*ff,0)); 
	end
	for j=1:i
		%Plot Trans
		dirTmp = nodePos(:,j)-nodePos(:,i);
		dirTmp = (norm(dirTmp)-2*nodeRadius) * dirTmp/norm(dirTmp);
		offTmp = [dirTmp(2); -dirTmp(1)] / norm(dirTmp);
		posTmp = nodePos(:,i) + nodeRadius * dirTmp/norm(dirTmp) + offTmp*0.05;
		plot2DArrowSC(posTmp, dirTmp, max([.8 .8 .8]-Trans(i,j)*ff,0)); 
	end
end

%Plot nodes
for i=1:nbStates
	a = linspace(0,2*pi,nodePts);
	meshTmp = [cos(a); sin(a)] * nodeRadius + repmat(nodePos(:,i),1,nodePts);
	patch(meshTmp(1,:), meshTmp(2,:), clrmap(i,:),'edgecolor',clrmap(i,:)*0.5, 'facealpha', valAlpha,'edgealpha', valAlpha);
	text(nodePos(1,i),nodePos(2,i),num2str(i),'HorizontalAlignment','center','FontWeight','bold','fontsize',20);
end

axis equal;

end

