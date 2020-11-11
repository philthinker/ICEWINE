function [probOut] = probNormalize(probIn)
%probNormalize Normalize the probabilities s.t. sum(probOut) = 1
%   probIn: N x 1, the probabilities to be normalized
%   probOut: N x 1, the normalized probabilities

tmpSum = sum(probIn(:,1),1);
probOut = probIn(:,1)./tmpSum;

end

