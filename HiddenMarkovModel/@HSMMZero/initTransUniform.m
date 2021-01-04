function obj = initTransUniform(obj)
%initTransUniform Init. the trans. param. uniformly.
%   @HSMMZero

obj.Trans = ones(obj.K,obj.K);
obj.Trans = obj.Trans./repmat(sum(obj.Trans,2),1,obj.K);
obj.StatePrior = ones(obj.K,1);
obj.StatePrior = obj.StatePrior/sum(obj.StatePrior);

end

