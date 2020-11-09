function [DataOut,logicIndices] = dataGap(range,t,Data)
%dataGap Set a gap in the Data. We assign NaN to those lost data.
%   range: 1 x 2, [leftLimit, rightLimit], leftLimit < rightLimit
%   t: N x 1, the time series
%   Data: N x D, the data
%   -------------------------------------------------
%   DataOut: N x D, the data output
%   logicIndices: N x 1 boolean, the logic indices of lost data ( true for NaN)

N = length(t);
tmpID = (1:N);
tmpID = tmpID(t > range(1) & t < range(2));

DataOut = Data;
DataOut(tmpID,:) = NaN;

logicIndices = isnan(DataOut(:,1));

end

