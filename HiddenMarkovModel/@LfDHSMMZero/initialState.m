function [StateID] = initialState(obj, currP,c)
%initialState Determining the initial state
%   currP: DP x 1, current position
%   c: Scalar > 0, safe factor (optional)
%   -----------------------------------------
%   StateID: Integer, the index of initial state
%   @LfDHSMMZero
DP = obj.DP;
currP = currP(1:DP,1);
if nargin < 3
    c = 2;
end
c = max(1, c);
% Determing S0 by Mahalanobis dist.
dState2 = zeros(1, obj.K);
for i = 1:K
    Xi = currP - obj.Mu(1:DP,i);
    Sigmai = obj.Sigma(1:DP, 1:DP, i);
    dState2(i) = c * (Xi' * Sigmai \ Xi);
end
[~, StateID] = min(dState2);
end

