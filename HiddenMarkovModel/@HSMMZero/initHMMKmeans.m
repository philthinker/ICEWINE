function obj = initHMMKmeans(obj,Demos)
%initHMMKMeans Init. the param. of HMM by K-Means algorithm
%   Demos: 1 x M cell of D x N data, demos
%   @HSMMZero

Data = obj.dataRegulate(Demos);
[idList,obj.Mu] = obj.kMeans(Data);  % K-Means clustering
for i = 1:obj.K
    idtmp = find(idList == i);
    obj.Prior(i) = length(idtmp);
    obj.Sigma(:,:,i) = cov([Data(:,idtmp) Data(:,idtmp)]');
    % Optional regularization term to avoid numerical instability
    obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(obj.D)*obj.params_diagRegFact_KMeans;
end
obj.Prior = obj.Prior/sum(obj.Prior);

end

