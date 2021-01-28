function [Mu,Sigma,obj] = prodLinearTransformedGauss(obj,frames)
%prodLinearTransformedGauss Product linear transformed Gaussians.
%   frames: 1 x F struct array:
%   |   A: D x D x F, A matrices
%   |   b: D x F, b vectors
%   -------------------------------------------------
%   Mu: D x K, the producted centers
%   Sigma: D x D x K, the producted covariances
%   obj
%   @TPTrajHSMMZero

for i=1:obj.K
    SigmaTmp = zeros(obj.D);
    MuTmp = zeros(obj.D,1);
    for m=1:obj.F
        MuP = frames(m).A * obj.Mus(:,m,i) + frames(m).b;
        SigmaP = frames(m).A * obj.Sigmas(:,:,m,i) * frames(m).A';
        SigmaTmp = SigmaTmp + inv(SigmaP);
        MuTmp = MuTmp + SigmaP\MuP;
    end
    obj.Sigma(:,:,i) = inv(SigmaTmp);
    obj.Mu(:,i) = obj.Sigma(:,:,i) * MuTmp;
end
Mu = obj.Mu;
Sigma = obj.Sigma;

end

