function [expData,expSigma] = combine(obj,Models,alpha)
%combine The combination operation of ProMP
%   Models: 1 x Nb ProMPZero array
%   alpha: (Nb+1) x N, the combination coefficent [0,1]
%   expData: D x N, expected data
%   expSigma: D x D x N, covariances
Nb = length(Models);
N = size(alpha,2);
[expData0,expSigma0] = obj.reproduct(N);
exps = [];
exps.data = expData0;
exps.Sigma = expSigma0;  % Note that here it is not Sigma^-1
exps = repmat(exps,[1,Nb+1]);
for i = 1:Nb
    [tmpData,tmpSigma] = Models(i+1).reproduct(N);
    exps(i).data = tmpData;
    exps(i).Sigma = tmpSigma;    % Note that here it is not Sigma^-1
end
expData = expData0;
expSigma = expSigma0;
for t = 1:N
    if alpha(1,t) > 0
        tmpExpSigmaInv = inv(exps(1).Sigma(:,:,t)./alpha(1,t));
        tmpExpData = tmpExpSigmaInv * exps(1).data(:,t);
    else
        % alpha_t == 0
        tmpExpSigmaInv = zeros(size(exps(1).Sigma(:,:,t)));
        tmpExpData = zeros(size(exp(1).data(:,t)));
    end
    for i = 2:Nb+1
        if alpha(i,t) > 0
            tmpExpSigmaInv_i = inv(exps(i).Sigma(:,:,t)./alpha(i,t));
        else
            % alpha_t == 0
            tmpExpSigmaInv_i = zeros(size(exps(i).Sigma(:,:,t)));
        end
        tmpExpSigmaInv = tmpExpSigmaInv + tmpExpSigmaInv_i;
        tmpExpData = tmpExpData + tmpExpSigmaInv_i * exps(i).data(:,t);
    end
    expSigma(:,:,t) = inv(tmpExpSigmaInv);
    expData(:,t) = tmpExpSigmaInv*tmpExpData;
end
end

