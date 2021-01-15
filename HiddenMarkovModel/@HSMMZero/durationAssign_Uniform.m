function [Pd, NGen] = durationAssign_Uniform(obj,N,c)
%durationAssign Assign the duration probabilities given the number of data
%   N: Integer, num. of data required
%   c: scalar, safety factor
%   -------------------------------------------------
%   Pd: K x N, duration probabilities
%   NGen: Integer, num. of data assigned (may not equal to N)
%   @HSMMZero

c = max([1, c(1,1)]);
N = round(max([obj.K,N(1,1)]));

% Number of maximum duration step to consider in the HSMM
if N <= obj.minSigmaPd / 2 * obj.K
    NGen = N;
else
    NGen = round(c * N/obj.K);
end

%Precomputation of duration probabilities 
Pd = zeros(obj.K, NGen);
if obj.logFlag
    % Log normal distribution
    for i=1:obj.K
        Pd(i,:) = obj.GaussPDF( log((1:NGen)), obj.MuPd(:,i), obj.SigmaPd(:,:,i) );
        %The rescaling formula below can be used to guarantee that the cumulated sum is one (to avoid the numerical issues)
        Pd(i,:) = Pd(i,:) / sum(Pd(i,:));
    end
else
    % Normal distribution
    for i=1:obj.K
        Pd(i,:) = obj.GaussPDF( (1:NGen), obj.MuPd(:,i), obj.SigmaPd(:,:,i) );
        %The rescaling formula below can be used to guarantee that the cumulated sum is one (to avoid the numerical issues)
        Pd(i,:) = Pd(i,:) / sum(Pd(i,:));
    end
end

end

