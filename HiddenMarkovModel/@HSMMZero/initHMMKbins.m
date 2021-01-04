function obj = initHMMKbins(obj,Demos)
%initHMMKbins Init. the param. of HMM by Kbins clustering
%   Demos: 1 x M cell of D x N data, demos
%   @HSMMZero

[Data,M] = obj.dataRegulate(Demos);
N = size(Demos{1},2);   %Delimit the cluster bins for the first demonstration
tSep = round(linspace(0, N, obj.K+1));

%Compute statistics for each bin
for i=1:obj.K
    id=[];
    for n=1:M
        id = [id (n-1)*N+[tSep(i)+1:tSep(i+1)]];
    end
    obj.Prior(i) = length(id);
    obj.Mu(:,i) = mean(Data(:,id),2);
    obj.Sigma(:,:,i) = cov(Data(:,id)') + eye(size(Data,1)) * obj.params_diagRegFact;
end
obj.Prior = obj.Prior / sum(obj.Prior);

end

