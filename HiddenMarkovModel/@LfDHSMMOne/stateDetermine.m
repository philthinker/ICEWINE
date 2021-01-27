function [SID] = stateDetermine(obj, p)
%stateDetermine Determining the related state
%   p: D x 1, position data
%   -------------------------------------------------
%   SID: Integer, the state ID
%   @LfDHSMMOne

K = obj.K;
D = obj.D;
p = p(1:obj.D,1);

closeness = zeros(K,1); % find "closest" state
for i=1:K
    closeness(i,:)=obj.GaussPDF(p, obj.Mu(1:D,i), obj.Sigma(1:D, 1:D, i));
end
[~,SID] = max(closeness);

end

