function obj = initTransLeftRight(obj,N)
%initTransLeftRight Init. the trans. param. via left-right model
%   N: Integer, num. of data
%   @HSMMZero

obj.Trans = zeros(obj.K);
for i = 1:obj.K-1
    obj.Trans(i,i) = 1 - obj.K/N;
    obj.Trans(i,i+1) = obj.K/N;
end
obj.Trans(obj.K,obj.K) = 1.0;
obj.StatePrior = zeros(obj.K,1);
obj.StatePrior(1) = 1;

% obj.Prior = ones(obj.K,1);

end

