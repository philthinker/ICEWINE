function [dynaData] = computeDynamicData(obj,data,dt,nDiff)
%computeDynamicData Compute the vel. and acc. given the pos. data. We
%always assume that the initial and final vel. and acc. are zero.
%   data: DPos x N, pos. data
%   dt: scalar, time difference
%   nDiff: integer > 1, DD
%   dynaData: DPos*DD x N, dynamic data
%   @TPGMMOne

N = size(data,2);
DPos = size(data,1);
DD = round(nDiff(1,1));
dynaData = repmat(data,[nDiff,1]);

if DD > 1
    for d = 2:DD
        tmpData = dynaData(DPos*(d-2)+1:DPos*(d-1),:);
        dynaData(DPos*(d-1)+1:DPos*d,1) = zeros(DPos,1);
        dynaData(DPos*(d-1)+1:DPos*d,2:end) = (tmpData(:,2:end) - tmpData(:,1:end-1))/dt;
    end
end

end

