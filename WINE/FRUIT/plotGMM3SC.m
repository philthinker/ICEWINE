function [h] = plotGMM3SC(Mu, Sigma, color, valAlpha, dispOpt)
%plotGMM3SC Plot the Gaussian mixture model in 3D view
%   Mu:         D x K array representing the centers of K Gaussians.
%   Sigma:      D x D x K array representing the covariance matrices of K Gaussians.
%   color:      1 x 3 array representing the RGB color to use for the display.
%   valAlpha:   scalar, transparency factor (optional).
%   dispOpt:    integer, 1 for GMM mode (default) others for GMR mode (optional).
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

if nargin<4
	valAlpha=1;
end
if nargin<5
	dispOpt=1;
end

nbData = size(Mu,2);
nbPoints = 20; %nb of points to form a circular path
nbRings = 10; %Number of circular paths following the principal direction

pts0 = [cos(linspace(0,2*pi,nbPoints)); sin(linspace(0,2*pi,nbPoints))];

h=[];
for n=1:nbData
  [V0,D0] = eigs(Sigma(:,:,n));
  U0 = real(V0*D0^.5);

  ringpts0 = [cos(linspace(0,pi,nbRings+1)); sin(linspace(0,pi,nbRings+1))];
  ringpts = zeros(3,nbRings); 
  ringpts([2,3],:) = ringpts0(:,1:nbRings);
  U = zeros(3); 
  U(:,[2,3]) = U0(:,[2,3]); 
  ringTmp = U*ringpts; 
 
  %Compute touching circular paths
  for j=1:nbRings
    U = zeros(3); 
    U(:,1) = U0(:,1); 
    U(:,2) = ringTmp(:,j);
    pts = zeros(3,nbPoints); 
    pts([1,2],:) = pts0;
    xring(:,:,j) = U*pts + repmat(Mu(:,n),1,nbPoints);
  end

  %Plot filled ellispoid
  xringfull = xring;
  xringfull(:,:,end+1) = xringfull(:,end:-1:1,1); %Close the ellipsoid
  for j=1:size(xringfull,3)-1
    for i=1:size(xringfull,2)-1
      xTmp = [xringfull(:,i,j) xringfull(:,i+1,j) xringfull(:,i+1,j+1) xringfull(:,i,j+1) xringfull(:,i,j)];
			if dispOpt==1
				%Version 1 (for GMM plots)
				h = [h patch(xTmp(1,:),xTmp(2,:),xTmp(3,:), min(color+0.1,1),'edgecolor',color,'linewidth',1,'facealpha',valAlpha,'edgealpha',valAlpha)]; %,'facealpha',0.5
			else
				%Version 2 (for GMR plots)
				patch(xTmp(1,:),xTmp(2,:),xTmp(3,:), min(color+0.35,1),'linestyle','none','facealpha',valAlpha);
			end
    end
  end
end

end

