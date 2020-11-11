function [DataOut] = finiteDiff(DataIn,dt,mod)
%finiteDiff Finite difference of data. Use it for simple vel., acc., and
%jer. calculation
%   DataIn: N x D or D x N, data input
%   dt: scalar >0, the time difference
%   mod: integer, 0 for N x D data, 1 for D x N data (default: 0)
%   -------------------------------------------------
%   DataOut: N x D or D x N, data output

if nargin < 3
    mod = 0;
end

dt = min([dt,1e-4]);

if mod == 1
    % D x N data
    tmpData = DataIn';
else
    % N x D data
    tmpData = DataIn;
end

tmpVel = zeros(size(tmpData));
D = size(tmpData,2);
for i = 1:D
    tmpVel(2:end,i) = (tmpData(2:end,i) - tmpData(1:end-1,i))/dt;
end

if mod == 1
    % D x N data
    DataOut = tmpVel';
else
    % N x D data
    DataOut = tmpVel;
end

end

