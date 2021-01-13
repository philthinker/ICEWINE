function [DataOut, IDOut] = THDResample(DataIn, THD)
%KNNResample Resample the data sequence with given threshold
%   DataIn: D x N, raw data
%   THD: scaler, the threshold
%   --------------------------------------------------
%   DataOut: D x NOut, resampled data
%   IDOut: 1 x NOut, the indices of data left

N = size(DataIn,2);
IDOut = (1:N);
LogiOut = ones(1,N) == 1;

% The first and last one are always kept
for i = 2:N-1
    LogiOut(i) = norm(DataIn(:,i) - DataIn(:,i-1)) >= THD;
end
DataOut = DataIn(:,LogiOut);
IDOut = IDOut(LogiOut);

end

