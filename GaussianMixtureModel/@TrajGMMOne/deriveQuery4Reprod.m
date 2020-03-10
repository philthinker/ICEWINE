function [query] = deriveQuery4Reprod(obj,Demo,N)
%deriveQuery4Reprod Derive the query sequence for demo reproduction. Note
%that the reproducted data are not identical to the demo data.
%   Demo: struct:
%   |   data: DPos x NDemo, demo data
%   N: integer, num. of data required (default: N = NDemo)
%   @TrajGMMOne

Data = Demo(1,1).data * obj.param_x_amplifier;
NDemo = size(Data,2);
DPos = obj.nVar/obj.nDiff;
K = obj.nKernel;
if nargin < 3
    N = NDemo;
end

if obj.tpFlag
    % TP-Traj-GMM
else
    % Ordinary Traj-GMM
    query = ones(1,N);
    queryData = zeros(DPos,N);
    x = linspace(0,1,NDemo); xq = linspace(0,1,N);
    for i = 1:DPos
        queryData(i,:) = pchip(x,Data(i,:),xq);
    end
    % Take the nearest Euclidean distance of position as classification
    % criteria
    Mu = obj.Mu(1:DPos,:);
    Dist = zeros(1,K);
    for i = 1:N
        for j = 1:K
            Dist(j) = norm(Mu(:,j) - queryData(:,i));
        end
        [~,query(i)] = min(Dist);
    end
end

end


