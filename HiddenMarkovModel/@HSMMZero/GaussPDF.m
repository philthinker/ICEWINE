function [prob] = GaussPDF(obj,Data, Mu, Sigma)
%GaussPDF Gaussian probabilistic distribution
%   Data: D x N, query data
%   Mu: D x 1, Mean
%   Sigma: D x D, Covariance
%   @HSMMZero

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

[nbVar,nbData] = size(Data);
Data = Data' - repmat(Mu',nbData,1);
prob = sum((Data/Sigma).*Data, 2);
prob = exp(-0.5*prob) / sqrt((2*pi)^nbVar * abs(det(Sigma)) + realmin);

end

