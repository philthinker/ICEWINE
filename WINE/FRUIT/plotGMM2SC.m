function [h] = plotGMM2SC(Mu, Sigma, color, valAlpha)
%plotGMM2SC Displays the parameters of a Gaussian Mixture Model (GMM) by
%plot. Never forget "hold on" in advance.
%	Mu:           D x K array representing the centers of K Gaussians.
%	Sigma:        D x D x K array representing the covariance matrices of K Gaussians.
%	color:        1 x 3 array representing the RGB color to use for the display.
%	valAlpha:     transparency factor (optional).
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

nbStates = size(Mu,2);
nbDrawingSeg = 35;
darkcolor = color*0.5; %max(color-0.5,0);
t = linspace(-pi, pi, nbDrawingSeg);

h=[];
for i=1:nbStates
	%R = real(sqrtm(1.0.*Sigma(:,:,i)));
	[V,D] = eig(Sigma(:,:,i));
	R = real(V*D.^.5);
	X = R * [cos(t); sin(t)] + repmat(Mu(:,i), 1, nbDrawingSeg);
	if nargin>3 %Plot with alpha transparency
		h = [h patch(X(1,:), X(2,:), color, 'lineWidth', 1, 'EdgeColor', darkcolor, 'facealpha', valAlpha,'edgealpha', valAlpha)];
		%MuTmp = [cos(t); sin(t)] * 0.3 + repmat(Mu(:,i),1,nbDrawingSeg);
		%h = [h patch(MuTmp(1,:), MuTmp(2,:), darkcolor, 'LineStyle', 'none', 'facealpha', valAlpha)];
		h = [h plot(Mu(1,:), Mu(2,:), '.', 'markersize', 6, 'color', darkcolor)];
	else %Plot without transparency
		h = [h patch(X(1,:), X(2,:), color, 'lineWidth', 1, 'EdgeColor', darkcolor)];
		h = [h plot(Mu(1,:), Mu(2,:), '.', 'markersize', 6, 'color', darkcolor)];
	end
end

end

