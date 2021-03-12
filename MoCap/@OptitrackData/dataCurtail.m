function [timeOut,dataOut] = dataCurtail(obj,range, data, mod)
%dataCurtail Curtail the data based on time series
%   range: 1 x 2, the time range, range(1) < range(2)
%   data: N x D or D x D x N, the data to processed
%   mod: integer, operation mode, 1 for relative time range out (default:0)
%   -------------------------------------------------
%   timeOut: Nout x 1, time output
%   dataOut: Nout x D, data output
%   @OptitrackData

if nargin < 4
    mod = 0;
end
timeOut = [];
dataOut = [];


range = range(1,:);
if range(1) > range(2)
    error('The left limit of time must less than the right one');
end

% Curtail
timeOut = obj.time;
N = size(obj.time,1);
tmpIndices = (1:N)';
tmpIndices = tmpIndices(timeOut > range(1));
timeOut = timeOut(tmpIndices);
tmpIndices = tmpIndices(timeOut < range(2));
timeOut = timeOut(timeOut < range(2));
% data type
if size(data,3) == N
    % D x D x N data
    dataOut = data(:,:,tmpIndices);
elseif size(data,1) == N
    % N x D data
    dataOut = data(tmpIndices,:);
else
    error('The size of data is not compatible to the time series');
end

if mod == 1
    % relative time series out
    timeOut = timeOut - timeOut(1,1);
else
    % absolute time series out
end

end

