function obj = initTransRandom(obj)
%initTransRandom Init. the trans. param. randomly.
%   @HSMMZero

obj.Trans = rand(obj.K,obj.K);
obj.Trans = obj.Trans./repmat(sum(obj.Trans,2),1,obj.K);
obj.StatePrior = rand(obj.K,1);
obj.StatePrior = obj.StatePrior/sum(obj.StatePrior);

end

