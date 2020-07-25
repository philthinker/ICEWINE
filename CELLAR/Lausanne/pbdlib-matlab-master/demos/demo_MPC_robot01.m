function demo_MPC_robot01
% Robot manipulator tracking task with MPC, by relinearization of the system plant at each time step.
%
% If this code is useful for your research, please cite the related publication:
% @incollection{Calinon19chapter,
% 	author="Calinon, S. and Lee, D.",
% 	title="Learning Control",
% 	booktitle="Humanoid Robotics: a Reference",
% 	publisher="Springer",
% 	editor="Vadakkepat, P. and Goswami, A.", 
% 	year="2019",
% 	pages="1261--1312",
% 	doi="10.1007/978-94-007-6046-2_68"
% }
% 
% The commented parts of this demo require the robotics toolbox RTB10 (http://petercorke.com/wordpress/toolboxes/robotics-toolbox).
% First run 'startup_rvc' from the robotics toolbox if you uncomment these parts.
%
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 50; %Number of datapoints in a trajectory
nbD = 10; %Time horizon
nbDOFs = 3; %Number of articulations

model.nbStates = 1; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data 
model.nbDeriv = 1; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVarX = model.nbVarPos * model.nbDeriv; %Dimension of state space
model.nbVarU = nbDOFs; %Dimension of control space

model.dt = 1E-3; %Time step duration
model.rfactor = 1E-2;	%Control cost in LQR

%Control cost matrix
R = eye(model.nbVarU) * model.rfactor;
R = kron(eye(nbD-1),R);

%Robot description
model.l = 0.6;

% L1 = Link('d', 0, 'a', model.l, 'alpha', 0);
% robot = SerialLink(repmat(L1,nbDOFs,1));


%% Task description
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.Mu = ones(model.nbVarPos,1) * .7; 
model.Sigma = eye(model.nbVarPos) * 1E-3;
MuQ = kron(ones(nbD,1), model.Mu);
Q = kron(eye(nbD), inv(model.Sigma)); 


%% Online batch LQR with relinearization of the system plant
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = ones(nbDOFs,1) * 0.1;
% Htmp = robot.fkine(q);
% x = Htmp.t(1:model.nbVarPos);
x = fkine(q, model);

for t=1:nbData	
% 	[Su, Sx] = computeTransferMatrices(q, model, robot, nbD); %Linearization of system plant
	[Su, Sx] = computeTransferMatrices(q, model, nbD); %Linearization of system plant
	u = (Su' * Q * Su + R) \ Su' * Q * (MuQ - Sx * x); %Control trajectory
	%Log data
	r(1).q(:,t) = q;
	r(1).x(:,t) = x;
	r(1).u(:,t) = u(1:model.nbVarU);
	%Update state
	q = q + u(1:model.nbVarU) * model.dt; 
% 	Htmp = robot.fkine(q); %Forward kinematics
% 	x = Htmp.t(1:model.nbVarPos); 
	x = fkine(q, model);
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 800 800],'color',[1 1 1],'name','x1-x2 plot'); hold on; axis off;
for t=round(linspace(1,nbData,2))
	colTmp = [.9,.9,.9] - [.7,.7,.7] * t/nbData;
	plotArm(r(1).q(:,t), ones(nbDOFs,1)*model.l, [0;0;-10+t*0.1], .06, colTmp);
end
% plotArm(model.Mu, ones(nbDOFs,1)*model.l, [0;0;10], .02, [.8 0 0]);

plotGMM(model.Mu, model.Sigma, [.8 0 0], .5);
plot(model.Mu(1,:), model.Mu(2,:), '.','markersize',20,'color',[.8 0 0]);
for n=1:1
	plot(r(n).x(1,:), r(n).x(2,:), '-','linewidth',2,'color',[0 0 0]);
	plot(r(n).x(1,:), r(n).x(2,:), '.','markersize',12,'color',[.2 .2 .2]);
end
axis equal; 


%% Timeline plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labList = {'$x_1$','$x_2$','$\dot{x}_1$','$\dot{x}_2$'};
figure('position',[830 10 800 800],'color',[1 1 1]); 
for j=1:model.nbVarX
	subplot(model.nbVarX+model.nbVarU,1,j); hold on;
	plot([1,nbData],[model.Mu(j) model.Mu(j)],':','color',[.5 .5 .5]);
	%Plot reproduction samples
	for n=1:1
		plot(r(n).x(j,:), '-','linewidth',1,'color',[0 0 0]); 
	end
	errorbar(nbData, model.Mu(j), model.Sigma(j,j)^.5, 'color',[.8 0 0]);
	plot(nbData, model.Mu(j), '.','markersize',15,'color',[.8 0 0]);
	ylabel(labList{j},'fontsize',14,'interpreter','latex');
end
for j=1:model.nbVarU
	subplot(model.nbVarX+model.nbVarU,1,model.nbVarX+j); hold on;
	%Plot reproduction samples
	for n=1:1
		plot(r(n).u(j,:), '-','linewidth',1,'color',[0 0 0]);
	end
	ylabel(['$u_' num2str(j) '$'],'fontsize',14,'interpreter','latex');
end
xlabel('$t$','fontsize',14,'interpreter','latex');

%print('-dpng','graphs/demo_MPC_robot01.png');
pause;
close all;
end

%%%%%%%%%%%%%%%%%%%%%%
function [Su, Sx] = transferMatrices(A, B, nbD)
	[nbVarX, nbVarU] = size(B);
	Su = zeros(nbVarX*nbD, nbVarU*(nbD-1));
	Sx = kron(ones(nbD,1), eye(nbVarX)); 
	M = B;
	for t=2:nbD
		id1 = (t-1)*nbVarX+1:nbD*nbVarX;
		Sx(id1,:) = Sx(id1,:) * A;
		id1 = (t-1)*nbVarX+1:t*nbVarX; 
		id2 = 1:(t-1)*nbVarU;
		Su(id1,id2) = M;
		M = [A*M(:,1:nbVarU), M]; %Also M = [A^(n-1)*B, M] or M = [Sx(id1,:)*B, M]
	end
end

%%%%%%%%%%%%%%%%%%%%%%
% function [Su, Sx, A, B] = computeTransferMatrices(q, model, robot, nbD)
function [Su, Sx, A, B] = computeTransferMatrices(q, model, nbD)
% 	J = robot.jacob0(q,'trans');
	J = jacob0(q, model);
	%Discrete linear system x(t+1) = x(t) + J * dq
	A = eye(model.nbVarX);
	B = J * model.dt;
	[Su, Sx] = transferMatrices(A, B, nbD);
end

%%%%%%%%%%%%%%%%%%%%%%
%Forward kinematics
function [x, Tf] = fkine(q, model)
	Tf = eye(4);
	T = repmat(Tf, [1,1,size(q,1)]);
	for n=1:size(q,1)
		c = cos(q(n));
		s = sin(q(n));
		T(:,:,n) = [c, -s, 0, model.l * c; ...
								s, c, 0, model.l * s; ...
								0, 0, 1, 0;
								0, 0, 0, 1]; %Homogeneous matrix 
		Tf = Tf * T(:,:,n);
	end
	x = Tf(1:2,end);
end

%%%%%%%%%%%%%%%%%%%%%%
%Jacobian with numerical computation
function J = jacob0(q, model)
	e = 1E-4;
	J = zeros(2,size(q,1));
	for n=1:size(q,1)
		qtmp = q;
		qtmp(n) = qtmp(n) + e;
		J(:,n) = (fkine(qtmp, model) - fkine(q, model)) / e;
	end
end