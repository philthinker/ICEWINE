function [MuOut, SigmaOut] = init_tensor_kmeans(obj, Data)
%init_tensor_kmeans Initialization of TP-TrajHSMM with k-means. Note that
%the obj.'s properties will not be modified.
%   Data: D x N, demo data
%   -------------------------------------------------
%   MuOut: D x F x K, centers
%   SigmaOut: D x D x F x K, covariances
%   @TPTrajHSMMZero

%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

diagRegularizationFactor = 1E-4; %Optional regularization term

nbVar = size(Data,1);
DataAll = reshape(Data, size(Data,1)*size(Data,2), size(Data,3)); %Matricization/flattening of tensor

%k-means clustering
[Data_id, Mu] = obj.kMeans(DataAll);

Sigma = zeros(nbVar*obj.F, nbVar*obj.F, obj.K);

for i=1:obj.K
	idtmp = find(Data_id==i);
	obj.Prior(i) = length(idtmp);
	Sigma(:,:,i) = cov([DataAll(:,idtmp) DataAll(:,idtmp)]') + eye(size(DataAll,1))*diagRegularizationFactor;
end
obj.Prior = obj.Prior / sum(obj.Prior);

%Reshape GMM parameters into tensor data
MuOut = zeros(nbVar,obj.F, obj.K);
SigmaOut = zeros(nbVar, nbVar, obj.F, obj.K);

for m=1:obj.F
	for i=1:obj.K
		MuOut(:,m,i) = Mu((m-1)*nbVar+1:m*nbVar,i);
		SigmaOut(:,:,m,i) = Sigma((m-1)*nbVar+1:m*nbVar,(m-1)*nbVar+1:m*nbVar,i);
	end
end
